#pragma once

// Loads reference geographic data for the airspace and airports layers.
//
// Airports are parsed from an OurAirports-format CSV (public-domain, stable
// URL — see cmake/deps/OpenAIP.cmake). Airspace polygons are parsed from
// GeoJSON (OpenAIP format or anything compatible). Both are optional —
// the loader silently returns empty containers when files are missing or
// malformed so the plugin still runs without them.

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

struct AirportFeature {
    std::string ident;         // ICAO code where available, otherwise OurAirports ident
    std::string iata;          // may be empty
    std::string name;          // human-readable airport name
    std::string municipality;  // city
    std::string country;       // ISO country code (e.g. "US", "DE")
    double lat = 0.0;
    double lon = 0.0;
    int    elevation_ft = 0;

    // OurAirports `type`: large_airport / medium_airport / small_airport /
    // heliport / seaplane_base / balloonport / closed.
    // We map this down to a compact size bucket for rendering.
    enum class Size { Large, Medium, Small, Heliport, Seaplane, Other };
    Size size = Size::Other;

    // Inferred from keywords / name heuristics (OurAirports has no explicit
    // military flag; a future OpenAIP-direct loader can set this precisely).
    bool  military = false;
};

struct AirspaceFeature {
    std::string name;
    // Airspace class bucket — collapsed from the ICAO/OpenAIP taxonomy into
    // the three rendering categories used by the airspace param enum.
    enum class Class {
        Controlled,   // FIR, CTR, TMA, Class A-E
        Restricted,   // P, R, D, MATZ, TSA, TRA
        Other
    };
    Class klass = Class::Other;

    // Size bucket drives zoom-based visibility. FIR covers a country; a
    // CTR covers an airport — showing both at the same zoom is noise.
    //   Large   FIR / UIR / CTA / ACC (country-scale)
    //   Medium  TMA / TRA / TSA / ADIZ / Airway (regional)
    //   Small   CTR / ATZ / MATZ / P / R / D / MOA / etc. (local)
    enum class Size { Large, Medium, Small, Other };
    Size size = Size::Other;

    // Short human-readable class tag used in tooltips: "FIR", "CTR", "TMA",
    // "R", "D", "P", "MATZ", etc. Preserves the OpenAIP sub-class after
    // the Class enum above folds it into a rendering bucket.
    std::string type_label;

    // Floor / ceiling in feet AMSL. 0 == ground, INT_MAX == unlimited.
    int floor_ft = 0;
    int ceiling_ft = 0;

    // Flat polygon (outer ring only — interior holes are rare in airspace and
    // we treat them as filled for rendering purposes).
    std::vector<std::pair<double, double>> ring_lat_lon;

    // Axis-aligned bbox of the ring, cached for viewport clipping.
    double min_lat = 0.0, max_lat = 0.0;
    double min_lon = 0.0, max_lon = 0.0;
};

struct ReferenceData {
    std::vector<AirportFeature>  airports;
    std::vector<AirspaceFeature> airspaces;
};

// Parse whichever files are present. Empty-string paths or missing files
// are fine — the returned vectors will just be empty.
ReferenceData load_reference_data(std::string_view airports_csv_path,
                                  std::string_view airspace_geojson_path);
