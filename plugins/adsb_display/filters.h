#pragma once

// Aircraft filter predicate. Takes the per-frame parameter snapshot so it can
// see every filter knob, and evaluates against a single AircraftView. Callers
// feed this into a one-shot pass over the aircraft vector so the map and the
// list panel agree on which aircraft are visible.

#include "adsb_display_state.h"
#include "geo_helpers.h"
#include "icao_country_db.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>

inline bool passes_filter(const AircraftView& ac, const MapDisplayState::Params& s,
                          double gps_lat, double gps_lon, bool has_gps)
{
    // altitude (airborne only — on-ground aircraft are usually alt 0 and
    // would be unfairly filtered out by a non-zero min). filter_alt_max == 0
    // means "no upper limit", matching the sentinel used by distance.
    if (!ac.on_ground) {
        if (ac.alt < s.filter_alt_min) return false;
        if (s.filter_alt_max > 0 && ac.alt > s.filter_alt_max) return false;
    } else if (!s.filter_show_ground) {
        return false;
    }

    // distance from receiver (km, requires GPS fix)
    if (has_gps && s.filter_distance_max_km > 0.0f) {
        double d = haversine_km(gps_lat, gps_lon, ac.lat, ac.lon);
        if (d > s.filter_distance_max_km) return false;
    }

    // military-only (adsb_classify_type: 1=military, 0=civilian, 2=unknown)
    if (s.filter_military_only) {
        if (adsb_classify_type(ac.category, ac.db_flags) != 1) return false;
    }

    // callsign substring (case-insensitive). Empty filter = no constraint.
    if (s.filter_callsign_substr[0] != '\0') {
        if (ac.callsign.empty()) return false;
        const char* needle = s.filter_callsign_substr;
        size_t nlen = std::strlen(needle);
        auto it = std::search(
            ac.callsign.begin(), ac.callsign.end(),
            needle, needle + nlen,
            [](char a, char b) {
                return std::toupper(static_cast<unsigned char>(a))
                     == std::toupper(static_cast<unsigned char>(b));
            });
        if (it == ac.callsign.end()) return false;
    }

    return true;
}
