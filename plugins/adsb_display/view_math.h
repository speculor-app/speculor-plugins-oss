#pragma once

// View-geometry helpers: relate the user-facing zoom / radius params
// to the internal viewport math. Separated from plugin.cpp so the
// event handler and the parameter setters can share them.

struct MapDisplayState;

// compute zoom level that fits a given lat/lon span into a viewport of
// size (w, h). Clamped to the plugin's zoom range [2, 18].
double zoom_for_span(double lat_span, double lon_span, int w, int h);

// convert a radius in km to approximate lat/lon spans at the given latitude.
void radius_to_span(double radius_km, double lat_deg,
                    double& lat_span, double& lon_span);

// recompute state radius from current zoom (keeps "Radius" param in sync
// when the user zooms the map). Reads/writes the viewport atomics; h is
// passed in since the canvas dimensions now live in Params (snapshot per
// frame) and these helpers run on multiple threads (radius depends on h only).
void sync_radius_from_zoom(MapDisplayState* s, int h);

// recompute state zoom from current radius (the inverse — used when the
// user edits the "Radius" param directly).
void sync_zoom_from_radius(MapDisplayState* s, int w, int h);
