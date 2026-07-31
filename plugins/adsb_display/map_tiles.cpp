#include "map_tiles.h"
#include "adsb_display_state.h"
#include "geo_helpers.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

namespace {

// Static gray square shown in place of a tile that hasn't downloaded yet.
const cv::Mat& placeholder_tile()
{
    static const cv::Mat p = make_placeholder();
    return p;
}

} // namespace

void composite_tiles(MapDisplayState* s, cv::Mat& canvas,
                     double zoom, double origin_tx, double origin_ty,
                     int w, int h)
{
    int tile_zoom = static_cast<int>(std::floor(zoom));
    double scale = std::exp2(zoom - tile_zoom);
    double eff_tile = k_tile_size * scale; // pixel size of one tile on screen

    // origin in tile_zoom integer-tile coordinates
    double origin_ix = origin_tx / scale;
    double origin_iy = origin_ty / scale;

    int tile_x_min = static_cast<int>(std::floor(origin_ix));
    int tile_y_min = static_cast<int>(std::floor(origin_iy));
    int tile_x_max = static_cast<int>(std::floor(origin_ix + w / eff_tile));
    int tile_y_max = static_cast<int>(std::floor(origin_iy + h / eff_tile));

    int max_tile = (1 << tile_zoom) - 1;

    for (int ty = tile_y_min; ty <= tile_y_max; ++ty) {
        for (int tx = tile_x_min; tx <= tile_x_max; ++tx) {
            int wrapped_x = ((tx % (max_tile + 1)) + (max_tile + 1)) % (max_tile + 1);
            if (ty < 0 || ty > max_tile) continue;

            TileKey key{wrapped_x, ty, tile_zoom};
            const cv::Mat* tile = s->tile_cache.get(key);

            if (!tile) {
                s->download_queue.request(key);
                tile = &placeholder_tile();
            }

            // pixel bounds of this tile on canvas
            double left = (tx - origin_ix) * eff_tile;
            double top  = (ty - origin_iy) * eff_tile;

            int dst_x = std::max(0, static_cast<int>(std::floor(left)));
            int dst_y = std::max(0, static_cast<int>(std::floor(top)));
            int dst_r = std::min(w, static_cast<int>(std::ceil(left + eff_tile)));
            int dst_b = std::min(h, static_cast<int>(std::ceil(top + eff_tile)));

            int copy_w = dst_r - dst_x;
            int copy_h = dst_b - dst_y;
            if (copy_w <= 0 || copy_h <= 0) continue;

            // corresponding source region in the 256x256 tile
            double inv_scale = k_tile_size / eff_tile;
            int src_x = static_cast<int>(std::round((dst_x - left) * inv_scale));
            int src_y = static_cast<int>(std::round((dst_y - top) * inv_scale));
            int src_w = static_cast<int>(std::round(copy_w * inv_scale));
            int src_h = static_cast<int>(std::round(copy_h * inv_scale));

            src_x = std::clamp(src_x, 0, k_tile_size - 1);
            src_y = std::clamp(src_y, 0, k_tile_size - 1);
            src_w = std::min(src_w, k_tile_size - src_x);
            src_h = std::min(src_h, k_tile_size - src_y);
            if (src_w <= 0 || src_h <= 0) continue;

            cv::Mat src_roi = (*tile)(cv::Rect(src_x, src_y, src_w, src_h));
            cv::Mat dst_roi = canvas(cv::Rect(dst_x, dst_y, copy_w, copy_h));

            if (src_w == copy_w && src_h == copy_h)
                src_roi.copyTo(dst_roi);
            else
                cv::resize(src_roi, dst_roi, dst_roi.size(), 0, 0, cv::INTER_LINEAR);
        }
    }
}
