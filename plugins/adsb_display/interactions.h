#pragma once

// Mouse / keyboard event handling for the map area: drag-to-pan,
// wheel-to-zoom (cursor-anchored), click to toggle info panel,
// right-click to add a waypoint, and delegation to the list panel
// when the cursor is over it.

#include <speculor/plugin_api.h>

#include <cstdint>

struct MapDisplayState;

// Returns the ICAO of the aircraft closest to the given canvas-pixel
// position within a 20 px hit radius, or 0 if none is close enough.
uint32_t hit_test_aircraft(MapDisplayState* s, float px_x, float px_y);

// Main event entry point. Returns 0 if handled, -1 otherwise.
int handle_event(MapDisplayState* s, const SpcInputEvent* event);
