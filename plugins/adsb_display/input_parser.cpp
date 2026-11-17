#include "input_parser.h"
#include "adsb_display_state.h"
#include "icao_country_db.h"
#include "unit_format.h"

#include <speculor/plugin_api.h>
#include <speculor/table_helpers.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <format>
#include <vector>

// aircraft_in fields, in the order the descriptor declares them
enum AircraftField : uint32_t {
    IN_ICAO_ADDR, IN_CALLSIGN, IN_ALT_BARO, IN_ALT_GEOM, IN_GROUND_SPEED, IN_IAS, IN_TAS,
    IN_MACH, IN_TRACK, IN_TRACK_RATE, IN_MAG_HEADING, IN_TRUE_HEADING, IN_ROLL, IN_BARO_RATE,
    IN_GEOM_RATE, IN_LATITUDE, IN_LONGITUDE, IN_NAV_ALT_MCP, IN_NAV_ALT_FMS, IN_NAV_HEADING,
    IN_NAV_QNH, IN_SQUAWK, IN_CATEGORY, IN_EMERGENCY, IN_SPI, IN_ALERT, IN_IS_ON_GROUND,
    IN_NIC, IN_NAC_P, IN_NAC_V, IN_SIL, IN_NIC_BARO, IN_GVA, IN_SDA, IN_DB_FLAGS, IN_MSG_SOURCE,
    IN_ADSB_VERSION, IN_RSSI, IN_MSG_COUNT, IN_SEEN, IN_SEEN_POS, IN_RC, IN_REGISTRATION,
    IN_TYPE_CODE, IN_AIRLINE_ICAO, IN_ORIGIN_ICAO, IN_DEST_ICAO,
};

// gps_in fields, in the order the descriptor declares them
enum GpsField : uint32_t { IN_GPS_LATITUDE, IN_GPS_LONGITUDE };

void resolve_offsets(MapDisplayState* s, const SpcTable* tbl)
{
    const uint32_t* off = tbl->field_offsets;
    s->off_icao      = off[IN_ICAO_ADDR];
    s->off_callsign  = off[IN_CALLSIGN];
    s->off_alt       = off[IN_ALT_BARO];
    s->off_gs        = off[IN_GROUND_SPEED];
    s->off_track     = off[IN_TRACK];
    s->off_lat       = off[IN_LATITUDE];
    s->off_lon       = off[IN_LONGITUDE];
    s->off_ground    = off[IN_IS_ON_GROUND];
    s->off_category  = off[IN_CATEGORY];
    s->off_db_flags  = off[IN_DB_FLAGS];
    s->off_emergency = off[IN_EMERGENCY];
    s->off_squawk    = off[IN_SQUAWK];
    s->off_rssi      = off[IN_RSSI];
    s->off_seen      = off[IN_SEEN];
    s->off_seen_pos  = off[IN_SEEN_POS];
    s->off_msg_count = off[IN_MSG_COUNT];
    s->off_ias       = off[IN_IAS];
    s->off_tas       = off[IN_TAS];
    s->off_mach      = off[IN_MACH];
    s->off_mag_heading  = off[IN_MAG_HEADING];
    s->off_true_heading = off[IN_TRUE_HEADING];
    s->off_baro_rate = off[IN_BARO_RATE];
    s->off_geom_rate = off[IN_GEOM_RATE];
    s->off_nav_alt_mcp = off[IN_NAV_ALT_MCP];
    s->off_nav_alt_fms = off[IN_NAV_ALT_FMS];
    s->off_msg_source = off[IN_MSG_SOURCE];
    s->off_spi       = off[IN_SPI];
    s->off_alert     = off[IN_ALERT];
    s->off_registration = off[IN_REGISTRATION];
    s->off_type_code = off[IN_TYPE_CODE];
    s->off_airline_icao = off[IN_AIRLINE_ICAO];
    s->off_origin_icao  = off[IN_ORIGIN_ICAO];
    s->off_dest_icao    = off[IN_DEST_ICAO];
}

bool read_gps_position(MapDisplayState* s, const SpcData* inputs,
                       uint32_t input_count,
                       double& gps_lat, double& gps_lon)
{
    if (input_count < 3 || inputs[2].type != SPC_DATA_TABLE || !inputs[2].table)
        return false;

    const SpcTable* tbl = inputs[2].table;
    if (tbl->record_count == 0) return false;

    const uint32_t off_lat = tbl->field_offsets[IN_GPS_LATITUDE];
    const uint32_t off_lon = tbl->field_offsets[IN_GPS_LONGITUDE];
    if (off_lat == SPC_NO_FIELD || off_lon == SPC_NO_FIELD)
        return false;

    gps_lat = spc_table_get_float64(tbl, 0, off_lat);
    gps_lon = spc_table_get_float64(tbl, 0, off_lon);

    // reject null island and out-of-range coords
    if (gps_lat == 0.0 && gps_lon == 0.0) return false;
    if (gps_lat < -90.0 || gps_lat > 90.0) return false;
    if (gps_lon < -180.0 || gps_lon > 180.0) return false;

    return true;
}

void parse_enrichment_record(MapDisplayState* s, const SpcData* inputs,
                             uint32_t input_count)
{
    // enrichment_in is port 1 (after aircraft_in=0, before gps_in=2)
    if (input_count < 2 || inputs[1].type != SPC_DATA_RECORD || !inputs[1].record)
        return;
    const char* json = inputs[1].record->json;
    if (!json || !json[0]) return;

    // Dedup against the last parsed JSON. Aircraft Enricher emits the
    // same blob every tick; at ~50 KB the parse + ~15 string allocs per
    // entry was the single biggest CPU sink in this plugin (13 ms/frame
    // of 19 ms total). A memcmp of the same bytes is ~3 µs.
    uint32_t length = inputs[1].record->length;
    if (length == 0) length = static_cast<uint32_t>(std::strlen(json));
    if (s->last_enrichment_json.size() == length &&
        std::memcmp(s->last_enrichment_json.data(), json, length) == 0) {
        return;
    }

    try {
        auto j = nlohmann::json::parse(json, json + length);
        if (!j.contains("enrichment") || !j["enrichment"].is_array()) return;

        for (const auto& entry : j["enrichment"]) {
            auto hex = entry.value("icao", "");
            if (hex.empty()) continue;
            uint32_t icao = std::stoul(hex, nullptr, 16);

            auto& e = s->enrichment_cache[icao];
            e.registration  = entry.value("registration", "");
            e.aircraft_type = entry.value("type", "");
            e.icao_type     = entry.value("icao_type", "");
            e.manufacturer  = entry.value("manufacturer", "");
            e.owner         = entry.value("owner", "");
            e.country       = entry.value("country", "");

            if (entry.contains("url_photo_thumbnail"))
                e.photo_thumb_url = entry.value("url_photo_thumbnail", "");
            if (entry.contains("url_photo"))
                e.photo_url = entry.value("url_photo", "");

            e.airline_name = entry.value("airline", "");
            e.airline_icao = entry.value("airline_icao", "");

            if (entry.contains("origin") && entry["origin"].is_object()) {
                auto& o = entry["origin"];
                e.origin_icao = o.value("icao", "");
                e.origin_iata = o.value("iata", "");
                e.origin_name = o.value("name", "");
                e.origin_municipality = o.value("city", "");
                e.origin_lat  = o.value("lat", 0.0);
                e.origin_lon  = o.value("lon", 0.0);
            }
            if (entry.contains("destination") && entry["destination"].is_object()) {
                auto& d = entry["destination"];
                e.dest_icao = d.value("icao", "");
                e.dest_iata = d.value("iata", "");
                e.dest_name = d.value("name", "");
                e.dest_municipality = d.value("city", "");
                e.dest_lat  = d.value("lat", 0.0);
                e.dest_lon  = d.value("lon", 0.0);
            }
        }
        // Only mark the JSON as parsed once we successfully consumed it —
        // a parse failure leaves the dedup snapshot stale on purpose so the
        // next valid blob (likely identical-but-fixed) parses through.
        s->last_enrichment_json.assign(json, length);
    } catch (...) {
        // silently ignore parse errors — enrichment is best-effort
    }
}

std::vector<AircraftView> read_aircraft_data(MapDisplayState* s,
                                             const SpcData* inputs,
                                             uint32_t input_count)
{
    std::vector<AircraftView> aircraft;
    if (input_count > 0 && inputs[0].type == SPC_DATA_TABLE && inputs[0].table) {
        const SpcTable* tbl = inputs[0].table;
        resolve_offsets(s, tbl);

        aircraft.reserve(tbl->record_count);
        for (uint32_t i = 0; i < tbl->record_count; ++i) {
            AircraftView ac;
            ac.icao     = spc_table_get_uint32(tbl, i, s->off_icao);
            ac.lat      = spc_table_get_float64(tbl, i, s->off_lat);
            ac.lon      = spc_table_get_float64(tbl, i, s->off_lon);
            ac.alt      = spc_table_get_int32(tbl, i, s->off_alt);
            ac.track    = spc_table_get_float(tbl, i, s->off_track);
            ac.gs       = spc_table_get_float(tbl, i, s->off_gs);
            ac.on_ground = spc_table_get_bool(tbl, i, s->off_ground);
            ac.callsign = spc_table_get_string(tbl, i, s->off_callsign);

            if (s->off_category != UINT32_MAX)
                ac.category = spc_table_get_uint8(tbl, i, s->off_category);
            if (s->off_db_flags != UINT32_MAX)
                ac.db_flags = spc_table_get_uint8(tbl, i, s->off_db_flags);
            if (s->off_emergency != UINT32_MAX)
                ac.emergency = spc_table_get_uint8(tbl, i, s->off_emergency);

            // extended fields for list panel and tooltip
            if (s->off_squawk != UINT32_MAX)
                ac.squawk = spc_table_get_uint16(tbl, i, s->off_squawk);
            if (s->off_rssi != UINT32_MAX)
                ac.rssi = spc_table_get_float(tbl, i, s->off_rssi);
            if (s->off_seen != UINT32_MAX)
                ac.seen = spc_table_get_float(tbl, i, s->off_seen);
            if (s->off_seen_pos != UINT32_MAX)
                ac.seen_pos = spc_table_get_float(tbl, i, s->off_seen_pos);
            if (s->off_msg_count != UINT32_MAX)
                ac.msg_count = spc_table_get_uint32(tbl, i, s->off_msg_count);
            if (s->off_ias != UINT32_MAX)
                ac.ias = spc_table_get_float(tbl, i, s->off_ias);
            if (s->off_tas != UINT32_MAX)
                ac.tas = spc_table_get_float(tbl, i, s->off_tas);
            if (s->off_mach != UINT32_MAX)
                ac.mach_num = spc_table_get_float(tbl, i, s->off_mach);
            if (s->off_mag_heading != UINT32_MAX)
                ac.mag_heading = spc_table_get_float(tbl, i, s->off_mag_heading);
            if (s->off_true_heading != UINT32_MAX)
                ac.true_heading = spc_table_get_float(tbl, i, s->off_true_heading);
            if (s->off_baro_rate != UINT32_MAX)
                ac.baro_rate = spc_table_get_int32(tbl, i, s->off_baro_rate);
            if (s->off_geom_rate != UINT32_MAX)
                ac.geom_rate = spc_table_get_int32(tbl, i, s->off_geom_rate);
            if (s->off_nav_alt_mcp != UINT32_MAX)
                ac.nav_alt_mcp = spc_table_get_int32(tbl, i, s->off_nav_alt_mcp);
            if (s->off_nav_alt_fms != UINT32_MAX)
                ac.nav_alt_fms = spc_table_get_int32(tbl, i, s->off_nav_alt_fms);
            if (s->off_msg_source != UINT32_MAX)
                ac.msg_source = spc_table_get_uint8(tbl, i, s->off_msg_source);
            if (s->off_spi != UINT32_MAX)
                ac.spi = spc_table_get_bool(tbl, i, s->off_spi);
            if (s->off_alert != UINT32_MAX)
                ac.alert = spc_table_get_bool(tbl, i, s->off_alert);
            if (s->off_registration != UINT32_MAX)
                ac.registration = spc_table_get_string(tbl, i, s->off_registration);
            if (s->off_type_code != UINT32_MAX)
                ac.type_code_str = spc_table_get_string(tbl, i, s->off_type_code);
            if (s->off_airline_icao != UINT32_MAX)
                ac.airline_icao = spc_table_get_string(tbl, i, s->off_airline_icao);
            if (s->off_origin_icao != UINT32_MAX)
                ac.origin_icao = spc_table_get_string(tbl, i, s->off_origin_icao);
            if (s->off_dest_icao != UINT32_MAX)
                ac.dest_icao = spc_table_get_string(tbl, i, s->off_dest_icao);

            // apply enrichment data from RECORD cache
            if (auto it = s->enrichment_cache.find(ac.icao); it != s->enrichment_cache.end()) {
                const auto& e = it->second;
                if (!e.origin_iata.empty()) ac.origin_iata = e.origin_iata;
                if (!e.dest_iata.empty())   ac.dest_iata = e.dest_iata;
                // prefer city name over full airport name
                if (!e.origin_municipality.empty())
                    ac.origin_name = e.origin_municipality;
                else if (!e.origin_name.empty())
                    ac.origin_name = e.origin_name;
                if (!e.dest_municipality.empty())
                    ac.dest_name = e.dest_municipality;
                else if (!e.dest_name.empty())
                    ac.dest_name = e.dest_name;
                if (!e.airline_name.empty())  ac.airline_name = e.airline_name;
                if (!e.manufacturer.empty())  ac.manufacturer = e.manufacturer;
                if (!e.aircraft_type.empty()) ac.aircraft_type = e.aircraft_type;
                if (!e.owner.empty())         ac.owner = e.owner;
                if (!e.photo_thumb_url.empty()) ac.photo_url = e.photo_thumb_url;
            }

            auto* country = icao_lookup_country(ac.icao);
            if (country) {
                ac.country_code = country->code;
                ac.country_name = country->name;
            }

            if (ac.lat == 0.0 && ac.lon == 0.0) continue;

            // Pre-format label strings once per frame so the per-aircraft
            // draw loop in aircraft_layer.cpp can blit them directly
            // (callsign + altitude + VS rate are each drawn twice — halo +
            // main — so caching halves the formatting work alongside the
            // allocation savings).
            if (ac.callsign.empty())
                ac.cs_label = std::format("{:06X}", ac.icao);
            else
                ac.cs_label = ac.callsign;
            if (ac.msg_source == 5)
                ac.cs_label = "M." + ac.cs_label;

            if (ac.alt != 0)
                ac.alt_label = format_altitude(ac.alt, s->cur.unit_system);

            if (s->cur.show_vs_indicator) {
                int32_t vs_fpm = ac.geom_rate != 0 ? ac.geom_rate : ac.baro_rate;
                if (std::abs(vs_fpm) >= 300) {
                    ac.vs_label = (s->cur.unit_system == 1)
                        ? std::format("{}", std::abs(fpm_to_mpm(vs_fpm)))
                        : std::format("{}", std::abs(vs_fpm));
                }
            }

            aircraft.push_back(std::move(ac));
        }
    }
    return aircraft;
}
