#pragma once

// Input-side helpers: schema-offset caching, table row -> AircraftView
// conversion, GPS position extraction, and enrichment-record parsing.

#include <speculor/plugin_api.h>

#include <vector>
#include <cstdint>

struct MapDisplayState;
struct AircraftView;

// Resolve and cache every aircraft-table field offset we care about.
// No-op after the first successful resolution (guarded by s->offsets_resolved).
void resolve_offsets(MapDisplayState* s, const SpcTable* tbl);

// Read GPS position from the third input table (port 2). Returns true if
// valid data was found and gps_lat/gps_lon are populated.
bool read_gps_position(MapDisplayState* s, const SpcData* inputs,
                       uint32_t input_count,
                       double& gps_lat, double& gps_lon);

// Parse the second input port (RECORD, port 1) into s->enrichment_cache.
// Silently ignores malformed JSON — enrichment is optional.
void parse_enrichment_record(MapDisplayState* s, const SpcData* inputs,
                             uint32_t input_count);

// Read the aircraft table (port 0) into a vector of AircraftViews, applying
// enrichment-cache data and ICAO country lookup. Aircraft without a valid
// position (lat==0 && lon==0) are dropped.
std::vector<AircraftView> read_aircraft_data(MapDisplayState* s,
                                             const SpcData* inputs,
                                             uint32_t input_count);
