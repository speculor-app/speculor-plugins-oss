#pragma once

// Airspace layer: renders FIR/CTR/TMA/P-R-D/MATZ polygons from the
// parsed OpenAIP GeoJSON onto the map. Each polygon is bbox-clipped to
// the viewport before projection — the full worldwide dataset can be
// tens of thousands of polygons, but any given frame only touches a few.

struct MapDisplayState;

// airspace_classes enum values (match the param descriptor)
enum {
    AIRSPACE_CLASSES_NONE       = 0,
    AIRSPACE_CLASSES_CONTROLLED = 1,
    AIRSPACE_CLASSES_RESTRICTED = 2,
    AIRSPACE_CLASSES_ALL        = 3,
};

void render_airspace_layer(MapDisplayState* s,
                           double zoom, double origin_tx, double origin_ty,
                           int map_w, int h);
