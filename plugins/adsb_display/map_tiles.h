#pragma once

// Tile layer: composite cached OSM tiles onto the canvas for the current
// viewport, and queue downloads for any missing tiles.

#include <opencv2/core.hpp>

struct MapDisplayState;

void composite_tiles(MapDisplayState* s, cv::Mat& canvas,
                     double zoom, double origin_tx, double origin_ty,
                     int w, int h);
