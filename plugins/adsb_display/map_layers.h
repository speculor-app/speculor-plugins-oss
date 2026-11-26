#pragma once

// Map-decoration layers: range rings centered on the GPS receiver,
// historical max-range circle, and the compass rose in the top-right.
// Each reads configuration from MapDisplayState and adds to `list`.

#include <draw/draw_list.h>

struct MapDisplayState;

// render concentric range rings centered on GPS position
void render_range_rings(MapDisplayState* s, spc::draw::DrawList& list,
                        double gps_lat, double gps_lon,
                        double zoom, double origin_tx, double origin_ty,
                        int w, int h);

// render max-range circle (dotted) at the farthest distance seen
void render_max_range(MapDisplayState* s, spc::draw::DrawList& list,
                      double gps_lat, double gps_lon,
                      double zoom, double origin_tx, double origin_ty,
                      int w, int h);

// compass rose in top-right corner of map area
void render_compass_rose(MapDisplayState* s, spc::draw::DrawList& list, int map_w, int h);
