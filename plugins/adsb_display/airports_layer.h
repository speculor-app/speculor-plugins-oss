#pragma once

// Airports layer: renders airport markers from the parsed OpenAIP /
// OurAirports data. Size bucket controls marker scaling; military
// airports get an extra red accent ring. When an aircraft is selected
// and has origin/destination ICAOs populated, this layer highlights
// those airports and draws a dashed line from the aircraft to its
// destination.

struct MapDisplayState;

// airport_min_type enum values (match the param descriptor)
enum {
    AIRPORT_MIN_LARGE_ONLY = 0,
    AIRPORT_MIN_MEDIUM     = 1,
    AIRPORT_MIN_ALL        = 2,
};

void render_airports_layer(MapDisplayState* s,
                           double zoom, double origin_tx, double origin_ty,
                           int map_w, int h);
