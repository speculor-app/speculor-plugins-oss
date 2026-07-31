#include "openaip_loader.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <climits>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>

// ── binary cache format ──────────────────────────────────────────────
// Parsing the JSON files takes 10-30 s even on a fast machine because
// nlohmann builds a full DOM and airports+airspaces together are ~340 MB.
// On the first load we build our compact in-memory representation and
// serialize it next to the source JSON; subsequent loads skip nlohmann
// entirely and read the binary (~25 MB total) in well under a second.
//
// The format is local-machine-only: native endianness, no versioning
// beyond a magic + schema-version header. Platform portability is not a
// goal — if the cache is stale we just regenerate it.

namespace {

constexpr uint32_t kAirportsMagic  = 0x41495250;   // 'AIRP'
constexpr uint32_t kAirspaceMagic  = 0x41535043;   // 'ASPC'
constexpr uint32_t kCacheVersion   = 3;            // bump on layout change

// I/O helpers ---------------------------------------------------------

void write_bytes(std::ofstream& f, const void* data, size_t n)
{
    f.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(n));
}
template <class T> void write_pod(std::ofstream& f, const T& v)
{
    static_assert(std::is_trivially_copyable_v<T>);
    write_bytes(f, &v, sizeof(T));
}
void write_str(std::ofstream& f, const std::string& s)
{
    uint16_t n = static_cast<uint16_t>(std::min<size_t>(s.size(), 0xFFFF));
    write_pod(f, n);
    if (n) write_bytes(f, s.data(), n);
}

bool read_bytes(std::ifstream& f, void* data, size_t n)
{
    f.read(reinterpret_cast<char*>(data), static_cast<std::streamsize>(n));
    return f.good();
}
template <class T> bool read_pod(std::ifstream& f, T& v)
{
    static_assert(std::is_trivially_copyable_v<T>);
    return read_bytes(f, &v, sizeof(T));
}
bool read_str(std::ifstream& f, std::string& s)
{
    uint16_t n;
    if (!read_pod(f, n)) return false;
    s.resize(n);
    if (n && !read_bytes(f, s.data(), n)) return false;
    return true;
}

bool cache_is_fresh(const std::filesystem::path& bin,
                    const std::filesystem::path& src)
{
    std::error_code ec;
    if (!std::filesystem::exists(bin, ec)) return false;
    if (!std::filesystem::exists(src, ec)) return true;  // only cache left, use it
    auto bin_t = std::filesystem::last_write_time(bin, ec);  if (ec) return false;
    auto src_t = std::filesystem::last_write_time(src, ec);  if (ec) return false;
    return bin_t >= src_t;
}

} // namespace

namespace {

// ── OpenAIP airport type enum ───────────────────────────────────────
// 0 = Airfield, 1 = GliderSite, 2 = AirfieldCivil, 3 = InternationalAirport,
// 4 = HeliportCivil, 5 = MilitaryAerodrome, 6 = LightAircraftField,
// 7 = AirfieldWater, 8 = UltraLightFlyingSite, 9 = HeliportMilitary,
// 10 = ClosedAirfield, 11 = SeaplaneBase

AirportFeature::Size size_from_openaip_type(int t)
{
    switch (t) {
        case 3:  return AirportFeature::Size::Large;    // International Airport
        case 2:  return AirportFeature::Size::Medium;   // Airfield Civil
        case 0:  return AirportFeature::Size::Medium;   // Airfield (generic)
        case 5:  return AirportFeature::Size::Medium;   // Military Aerodrome — size-wise treat as medium
        case 6:  return AirportFeature::Size::Small;    // Light Aircraft Field
        case 1:  return AirportFeature::Size::Small;    // Glider Site
        case 8:  return AirportFeature::Size::Small;    // Ultra-light
        case 4:  return AirportFeature::Size::Heliport; // Heliport Civil
        case 9:  return AirportFeature::Size::Heliport; // Heliport Military
        case 7:  return AirportFeature::Size::Seaplane; // Airfield Water
        case 11: return AirportFeature::Size::Seaplane; // Seaplane Base
        case 10: return AirportFeature::Size::Other;    // Closed — not shown
        default: return AirportFeature::Size::Other;
    }
}

bool is_military_openaip_type(int t)
{
    return t == 5 || t == 9;  // MilitaryAerodrome or HeliportMilitary
}

// ── OpenAIP airspace type + icaoClass -> our three buckets ─────────
// Airspace type enum (from OpenAIP docs, 2024 schema):
//   0=Other 1=Restricted 2=Danger 3=Prohibited 4=CTR 5=TMZ 6=RMZ 7=TMA
//   8=TRA 9=TSA 10=FIR 11=UIR 12=ADIZ 13=ATZ 14=MATZ 15=Airway
//   16=MTR 17=AlertArea 18=WarningArea 19=ProtectedArea
//   20=HTZ 21=GliderSector 22=TIZ 23=TIA 24=MTA 25=CTA 26=ACC 27=RecreationalArea
// icaoClass enum: 0=A 1=B 2=C 3=D 4=E 5=F 6=G 7=SUA 8=Other

// Size bucket for zoom gating. Categorization is conservative: only the
// two ATC-regional types (TMA, ADIZ) stay "always visible" at Medium.
// Everything else localized — CTRs, military training areas, corridors,
// training sectors — goes to Small and only unlocks at higher zoom,
// otherwise a continental view layers dozens of polygons on top of each
// other and the whole thing reads as noise.
AirspaceFeature::Size size_of_openaip_airspace(int type)
{
    switch (type) {
        case 10: case 11: case 25: case 26:  // FIR, UIR, CTA, ACC
            return AirspaceFeature::Size::Large;
        case 7:  case 12:                    // TMA, ADIZ
            return AirspaceFeature::Size::Medium;
        case 1:  case 2:  case 3:            // R, D, P
        case 4:  case 5:  case 6:            // CTR, TMZ, RMZ
        case 8:  case 9:                     // TRA, TSA
        case 13: case 14:                    // ATZ, MATZ
        case 15: case 16:                    // Airway, MTR
        case 17: case 18:                    // Alert, Warning
        case 19: case 20: case 21:           // Protected, HTZ, GliderSector
        case 22: case 23: case 24:           // TIZ, TIA, MTA
        case 27:                             // Recreational
            return AirspaceFeature::Size::Small;
        default:
            return AirspaceFeature::Size::Other;
    }
}

// Short textual tag — used in the click info panel so users can tell a
// MATZ from a TMZ.
const char* type_label_of_openaip_airspace(int type)
{
    switch (type) {
        case 0:  return "Other";
        case 1:  return "R";
        case 2:  return "D";
        case 3:  return "P";
        case 4:  return "CTR";
        case 5:  return "TMZ";
        case 6:  return "RMZ";
        case 7:  return "TMA";
        case 8:  return "TRA";
        case 9:  return "TSA";
        case 10: return "FIR";
        case 11: return "UIR";
        case 12: return "ADIZ";
        case 13: return "ATZ";
        case 14: return "MATZ";
        case 15: return "AWY";
        case 16: return "MTR";
        case 17: return "Alert";
        case 18: return "Warning";
        case 19: return "Protected";
        case 20: return "HTZ";
        case 21: return "Glider";
        case 22: return "TIZ";
        case 23: return "TIA";
        case 24: return "MTA";
        case 25: return "CTA";
        case 26: return "ACC";
        case 27: return "RecArea";
        default: return "?";
    }
}

AirspaceFeature::Class classify_openaip_airspace(int type, int icao_class)
{
    // Any ICAO class A-G (0-6) is controlled airspace by definition.
    if (icao_class >= 0 && icao_class <= 6)
        return AirspaceFeature::Class::Controlled;

    switch (type) {
        case 4:  // CTR
        case 7:  // TMA
        case 10: // FIR
        case 11: // UIR
        case 13: // ATZ
        case 25: // CTA
        case 26: // ACC
            return AirspaceFeature::Class::Controlled;

        case 1:  // Restricted
        case 2:  // Danger
        case 3:  // Prohibited
        case 8:  // TRA
        case 9:  // TSA
        case 12: // ADIZ
        case 14: // MATZ
        case 16: // MTR
        case 17: // Alert
        case 18: // Warning
        case 24: // MTA
            return AirspaceFeature::Class::Restricted;

        default:
            return AirspaceFeature::Class::Other;
    }
}

// OpenAIP altitude format: { value: N, unit: U, referenceDatum: R }.
// Units (best-known mapping): 0=m 1=ft 6=FL. Convert everything to feet
// so the rest of the pipeline can reason in a single unit.
int parse_altitude(const nlohmann::json& v)
{
    if (!v.is_object()) return 0;
    double val = v.value("value", 0.0);
    int unit = v.value("unit", 1);
    switch (unit) {
        case 0:  return static_cast<int>(val * 3.28084);   // m -> ft
        case 6:  return static_cast<int>(val * 100.0);     // FL -> ft
        case 1:  // ft
        default: return static_cast<int>(val);
    }
}

void compute_bbox(AirspaceFeature& a)
{
    if (a.ring_lat_lon.empty()) return;
    a.min_lat = a.max_lat = a.ring_lat_lon[0].first;
    a.min_lon = a.max_lon = a.ring_lat_lon[0].second;
    for (const auto& [lat, lon] : a.ring_lat_lon) {
        a.min_lat = std::min(a.min_lat, lat);
        a.max_lat = std::max(a.max_lat, lat);
        a.min_lon = std::min(a.min_lon, lon);
        a.max_lon = std::max(a.max_lon, lon);
    }
}

// ── airports.json: JSON array of OpenAIP airport records ─────────

void load_airports(const std::filesystem::path& path,
                   std::vector<AirportFeature>& out)
{
    std::ifstream f(path);
    if (!f) return;

    nlohmann::json j;
    try { f >> j; } catch (...) { return; }
    if (!j.is_array()) return;

    out.reserve(out.size() + j.size());
    for (const auto& rec : j) {
        int type = rec.value("type", -1);
        if (type == 10) continue;  // closed — skip entirely

        AirportFeature a;
        a.size     = size_from_openaip_type(type);
        a.military = is_military_openaip_type(type);
        a.name     = rec.value("name", "");
        a.country  = rec.value("country", "");
        a.ident    = rec.value("icaoCode", "");
        a.iata     = rec.value("iataCode", "");
        // Fallback identifier when ICAO is empty — the mongo _id is stable
        // but ugly; it's still better than nothing for dedup/highlight lookup.
        if (a.ident.empty()) a.ident = rec.value("_id", "");

        if (!rec.contains("geometry")) continue;
        const auto& g = rec["geometry"];
        if (g.value("type", "") != "Point") continue;
        if (!g.contains("coordinates") || !g["coordinates"].is_array()
            || g["coordinates"].size() < 2) continue;
        a.lon = g["coordinates"][0].get<double>();
        a.lat = g["coordinates"][1].get<double>();

        if (rec.contains("elevation") && rec["elevation"].is_object()) {
            const auto& e = rec["elevation"];
            double v = e.value("value", 0.0);
            int u = e.value("unit", 1);
            a.elevation_ft = (u == 0)
                ? static_cast<int>(v * 3.28084)
                : static_cast<int>(v);
        }

        if (a.size == AirportFeature::Size::Other) continue;  // skip unclassified
        out.push_back(std::move(a));
    }
}

// ── airspace.json: JSON array of OpenAIP airspace records ────────

void load_airspaces(const std::filesystem::path& path,
                    std::vector<AirspaceFeature>& out)
{
    std::ifstream f(path);
    if (!f) return;

    nlohmann::json j;
    try { f >> j; } catch (...) { return; }
    if (!j.is_array()) return;

    out.reserve(out.size() + j.size());
    for (const auto& rec : j) {
        AirspaceFeature a;
        a.name = rec.value("name", "");

        int type = rec.value("type", 0);
        int icao_class = rec.value("icaoClass", -1);
        a.klass = classify_openaip_airspace(type, icao_class);
        a.size  = size_of_openaip_airspace(type);
        a.type_label = type_label_of_openaip_airspace(type);

        if (rec.contains("lowerLimit")) a.floor_ft   = parse_altitude(rec["lowerLimit"]);
        if (rec.contains("upperLimit")) a.ceiling_ft = parse_altitude(rec["upperLimit"]);

        if (!rec.contains("geometry")) continue;
        const auto& g = rec["geometry"];
        const auto gtype = g.value("type", "");
        const nlohmann::json* ring = nullptr;
        if (gtype == "Polygon" && g.contains("coordinates")
            && g["coordinates"].is_array() && !g["coordinates"].empty()) {
            ring = &g["coordinates"][0];
        } else if (gtype == "MultiPolygon" && g.contains("coordinates")
                   && g["coordinates"].is_array() && !g["coordinates"].empty()
                   && g["coordinates"][0].is_array() && !g["coordinates"][0].empty()) {
            ring = &g["coordinates"][0][0];
        }
        if (!ring || !ring->is_array() || ring->size() < 3) continue;

        a.ring_lat_lon.reserve(ring->size());
        for (const auto& pt : *ring) {
            if (!pt.is_array() || pt.size() < 2) continue;
            // GeoJSON order: [lon, lat]
            double lon = pt[0].get<double>();
            double lat = pt[1].get<double>();
            a.ring_lat_lon.emplace_back(lat, lon);
        }
        if (a.ring_lat_lon.size() < 3) continue;

        compute_bbox(a);
        out.push_back(std::move(a));
    }
}

} // namespace

namespace {

// ── airport cache ────────────────────────────────────────────────────

bool load_airports_bin(const std::filesystem::path& path,
                       std::vector<AirportFeature>& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;

    uint32_t magic, version;
    uint64_t count;
    if (!read_pod(f, magic) || magic != kAirportsMagic) return false;
    if (!read_pod(f, version) || version != kCacheVersion) return false;
    if (!read_pod(f, count)) return false;

    out.clear();
    out.reserve(static_cast<size_t>(count));
    for (uint64_t i = 0; i < count; ++i) {
        AirportFeature a;
        if (!read_str(f, a.ident)) return false;
        if (!read_str(f, a.iata)) return false;
        if (!read_str(f, a.name)) return false;
        if (!read_str(f, a.municipality)) return false;
        if (!read_str(f, a.country)) return false;
        if (!read_pod(f, a.lat)) return false;
        if (!read_pod(f, a.lon)) return false;
        if (!read_pod(f, a.elevation_ft)) return false;
        uint8_t size_byte, mil_byte;
        if (!read_pod(f, size_byte)) return false;
        if (!read_pod(f, mil_byte)) return false;
        a.size = static_cast<AirportFeature::Size>(size_byte);
        a.military = (mil_byte != 0);
        out.push_back(std::move(a));
    }
    return true;
}

void save_airports_bin(const std::filesystem::path& path,
                       const std::vector<AirportFeature>& in)
{
    std::filesystem::path tmp = path;
    tmp += ".tmp";
    {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        if (!f) return;
        write_pod(f, kAirportsMagic);
        write_pod(f, kCacheVersion);
        write_pod(f, static_cast<uint64_t>(in.size()));
        for (const auto& a : in) {
            write_str(f, a.ident);
            write_str(f, a.iata);
            write_str(f, a.name);
            write_str(f, a.municipality);
            write_str(f, a.country);
            write_pod(f, a.lat);
            write_pod(f, a.lon);
            write_pod(f, a.elevation_ft);
            write_pod(f, static_cast<uint8_t>(a.size));
            write_pod(f, static_cast<uint8_t>(a.military ? 1 : 0));
        }
    }
    std::error_code ec;
    std::filesystem::rename(tmp, path, ec);
    if (ec) std::filesystem::remove(tmp, ec);
}

// ── airspace cache ───────────────────────────────────────────────────

bool load_airspaces_bin(const std::filesystem::path& path,
                        std::vector<AirspaceFeature>& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;

    uint32_t magic, version;
    uint64_t count;
    if (!read_pod(f, magic) || magic != kAirspaceMagic) return false;
    if (!read_pod(f, version) || version != kCacheVersion) return false;
    if (!read_pod(f, count)) return false;

    out.clear();
    out.reserve(static_cast<size_t>(count));
    for (uint64_t i = 0; i < count; ++i) {
        AirspaceFeature a;
        if (!read_str(f, a.name)) return false;
        uint8_t klass;
        if (!read_pod(f, klass)) return false;
        a.klass = static_cast<AirspaceFeature::Class>(klass);
        uint8_t size_byte;
        if (!read_pod(f, size_byte)) return false;
        a.size = static_cast<AirspaceFeature::Size>(size_byte);
        if (!read_str(f, a.type_label)) return false;
        if (!read_pod(f, a.floor_ft)) return false;
        if (!read_pod(f, a.ceiling_ft)) return false;
        if (!read_pod(f, a.min_lat)) return false;
        if (!read_pod(f, a.max_lat)) return false;
        if (!read_pod(f, a.min_lon)) return false;
        if (!read_pod(f, a.max_lon)) return false;
        uint32_t pt_count;
        if (!read_pod(f, pt_count)) return false;
        a.ring_lat_lon.resize(pt_count);
        if (pt_count && !read_bytes(f, a.ring_lat_lon.data(),
                                    pt_count * sizeof(std::pair<double, double>))) return false;
        out.push_back(std::move(a));
    }
    return true;
}

void save_airspaces_bin(const std::filesystem::path& path,
                        const std::vector<AirspaceFeature>& in)
{
    std::filesystem::path tmp = path;
    tmp += ".tmp";
    {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        if (!f) return;
        write_pod(f, kAirspaceMagic);
        write_pod(f, kCacheVersion);
        write_pod(f, static_cast<uint64_t>(in.size()));
        for (const auto& a : in) {
            write_str(f, a.name);
            write_pod(f, static_cast<uint8_t>(a.klass));
            write_pod(f, static_cast<uint8_t>(a.size));
            write_str(f, a.type_label);
            write_pod(f, a.floor_ft);
            write_pod(f, a.ceiling_ft);
            write_pod(f, a.min_lat);
            write_pod(f, a.max_lat);
            write_pod(f, a.min_lon);
            write_pod(f, a.max_lon);
            uint32_t pt_count = static_cast<uint32_t>(a.ring_lat_lon.size());
            write_pod(f, pt_count);
            if (pt_count) write_bytes(f, a.ring_lat_lon.data(),
                                      pt_count * sizeof(std::pair<double, double>));
        }
    }
    std::error_code ec;
    std::filesystem::rename(tmp, path, ec);
    if (ec) std::filesystem::remove(tmp, ec);
}

} // namespace

ReferenceData load_reference_data(std::string_view airports_path,
                                  std::string_view airspace_path)
{
    ReferenceData out;
    namespace fs = std::filesystem;

    // ── airports: binary cache fast-path, JSON fall-back ──────────────
    if (!airports_path.empty()) {
        fs::path json_p{airports_path};
        fs::path bin_p = json_p; bin_p.replace_extension(".bin");
        if (cache_is_fresh(bin_p, json_p) && load_airports_bin(bin_p, out.airports)) {
            // loaded from cache — nothing more to do
        } else if (fs::exists(json_p)) {
            load_airports(json_p, out.airports);
            save_airports_bin(bin_p, out.airports);
        }
    }

    // ── airspace: same pattern ────────────────────────────────────────
    if (!airspace_path.empty()) {
        fs::path json_p{airspace_path};
        fs::path bin_p = json_p; bin_p.replace_extension(".bin");
        if (cache_is_fresh(bin_p, json_p) && load_airspaces_bin(bin_p, out.airspaces)) {
            // loaded from cache
        } else if (fs::exists(json_p)) {
            load_airspaces(json_p, out.airspaces);
            save_airspaces_bin(bin_p, out.airspaces);
        }
    }

    return out;
}
