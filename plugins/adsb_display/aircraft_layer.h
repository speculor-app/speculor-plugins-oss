#pragma once

// Aircraft-layer rendering: altitude-colored trails, trend vectors,
// staleness fade, selected + emergency + MLAT rings, icon silhouettes,
// callsign + altitude labels, and vertical-speed glyphs.
//
// Also publishes per-frame aircraft screen positions to the state's
// hit-test buffer so the event handler can do click/hover lookups.

#include <vector>

struct MapDisplayState;
struct AircraftView;

void render_aircraft_layer(MapDisplayState* s,
                           const std::vector<AircraftView>& aircraft,
                           double zoom, double origin_tx, double origin_ty,
                           int w, int h);
