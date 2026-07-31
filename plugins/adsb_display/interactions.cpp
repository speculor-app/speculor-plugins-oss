#include "interactions.h"
#include "adsb_display_state.h"
#include "aircraft_list_panel.h"
#include "geo_helpers.h"
#include "view_math.h"
#include "waypoint_delete_confirm.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <mutex>

// Adjust the list-panel scroll so the row for icao is on-screen. No-op when
// the icao isn't in the rendered subset (filter excluded it / list hidden).
static void scroll_list_to_icao(MapDisplayState* s, uint32_t icao)
{
    // event thread: read GUI params via a SharedParams snapshot (s->cur is
    // worker-owned and unsynchronized w.r.t. this thread).
    const auto p = s->params.snapshot();
    if (icao == 0 || !p.show_list) return;

    int row = -1;
    for (size_t i = 0; i < s->panel_aircraft.size(); ++i) {
        if (s->panel_aircraft[i].icao == icao) { row = static_cast<int>(i); break; }
    }
    if (row < 0) return;

    int data_h = p.height - adsb_list::HEADER_H - adsb_list::STATUS_BAR;
    int visible = data_h / adsb_list::ROW_H;
    if (visible <= 0) return;

    int& off = s->list_state.scroll_offset;
    if (row < off) {
        off = row;
    } else if (row >= off + visible) {
        off = row - visible + 1;
    }
}

uint32_t hit_test_aircraft(MapDisplayState* s, float px_x, float px_y)
{
    int active = s->screen_pos_active.load(std::memory_order_acquire);
    const auto& positions = s->screen_pos[active];

    uint32_t best_icao = 0;
    float best_dist_sq = 20.0f * 20.0f;  // 20 px hit radius
    for (const auto& sp : positions) {
        float dx = px_x - sp.px_x;
        float dy = px_y - sp.px_y;
        float d2 = dx * dx + dy * dy;
        if (d2 < best_dist_sq) {
            best_dist_sq = d2;
            best_icao = sp.icao;
        }
    }
    return best_icao;
}

// Waypoint hit-test — projects each user-placed waypoint to canvas pixels
// (waypoints are stored as lat/lon, no cached screen-pos buffer like
// aircraft / airports) and picks the closest within 18 px of the cursor.
static int32_t hit_test_waypoint(MapDisplayState* s,
                                 double zoom, double origin_tx, double origin_ty,
                                 float px_x, float px_y)
{
    constexpr double hit_radius_px_sq = 18.0 * 18.0;
    int32_t best_idx = -1;
    double best_dist_sq = hit_radius_px_sq;
    std::lock_guard lock(s->waypoints_mutex);
    for (size_t i = 0; i < s->waypoints.size(); ++i) {
        const auto& wp = s->waypoints[i];
        auto p = geo_to_pixel(wp.lat, wp.lon, zoom, origin_tx, origin_ty);
        double dx = p.x - px_x;
        double dy = p.y - px_y;
        double d2 = dx * dx + dy * dy;
        if (d2 < best_dist_sq) {
            best_dist_sq = d2;
            best_idx = static_cast<int32_t>(i);
        }
    }
    return best_idx;
}

// Airport hit-test — tighter 15 px radius than aircraft since airports pack
// densely in metro areas and we don't want clicks latching onto random
// nearby heliports.
static int32_t hit_test_airport(MapDisplayState* s, float px_x, float px_y)
{
    int active = s->airport_screen_pos_active.load(std::memory_order_acquire);
    const auto& positions = s->airport_screen_pos[active];

    int32_t best_idx = -1;
    float best_dist_sq = 15.0f * 15.0f;
    for (const auto& sp : positions) {
        float dx = px_x - sp.px_x;
        float dy = px_y - sp.px_y;
        float d2 = dx * dx + dy * dy;
        if (d2 < best_dist_sq) {
            best_dist_sq = d2;
            best_idx = sp.airport_idx;
        }
    }
    return best_idx;
}

// Ray-cast point-in-polygon (even-odd rule). Works on any 2D coordinates
// — we use lat/lon directly since the ring is stored that way.
static bool point_in_polygon(double lat, double lon,
                             const std::vector<std::pair<double, double>>& ring)
{
    bool inside = false;
    size_t n = ring.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        double lat_i = ring[i].first,  lon_i = ring[i].second;
        double lat_j = ring[j].first,  lon_j = ring[j].second;
        if ((lat_i > lat) != (lat_j > lat) &&
            lon < (lon_j - lon_i) * (lat - lat_i) / (lat_j - lat_i) + lon_i)
            inside = !inside;
    }
    return inside;
}

// Airspace hit-test — reverse-project the click back to lat/lon and run
// point-in-polygon against every airspace that rendered this frame. When
// multiple polygons contain the cursor (e.g. a CTR inside a TMA inside a
// FIR) we pick the one with the smallest bbox so the user gets the most
// specific airspace, not the outermost container.
static int32_t hit_test_airspace(MapDisplayState* s,
                                 double zoom, double origin_tx, double origin_ty,
                                 float px_x, float px_y)
{
    if (!s->reference_loaded.load(std::memory_order_acquire)) return -1;

    double cursor_lat, cursor_lon;
    pixel_to_geo(px_x, px_y, zoom, origin_tx, origin_ty, cursor_lat, cursor_lon);

    int active = s->visible_airspace_active.load(std::memory_order_acquire);
    const auto& visible = s->visible_airspace_idx[active];

    int32_t best_idx = -1;
    double best_area = std::numeric_limits<double>::infinity();
    for (int32_t idx : visible) {
        if (idx < 0 || static_cast<size_t>(idx) >= s->reference.airspaces.size()) continue;
        const auto& a = s->reference.airspaces[idx];
        if (cursor_lat < a.min_lat || cursor_lat > a.max_lat) continue;
        if (cursor_lon < a.min_lon || cursor_lon > a.max_lon) continue;
        if (!point_in_polygon(cursor_lat, cursor_lon, a.ring_lat_lon)) continue;

        double area = (a.max_lat - a.min_lat) * (a.max_lon - a.min_lon);
        if (area < best_area) {
            best_area = area;
            best_idx = idx;
        }
    }
    return best_idx;
}

// Modal handler for the right-click delete confirmation. While active, every
// other map interaction is gated — the user must explicitly Cancel or Delete
// before resuming normal use of the map.
static int handle_delete_confirm_event(MapDisplayState* s, const SpcInputEvent* event)
{
    const auto p = s->params.snapshot();
    int w = p.width;
    int h = p.height;

    // ESC cancels.
    if (event->type == SPC_INPUT_EVENT_KEY_PRESS
        && event->key.key_code == SPC_KEY_ESCAPE) {
        s->delete_confirm_idx.store(-1, std::memory_order_relaxed);
        return 0;
    }

    if (event->type == SPC_INPUT_EVENT_MOUSE_PRESS) {
        // Right-click anywhere cancels.
        if (event->mouse.button & SPC_MOUSE_BUTTON_RIGHT) {
            s->delete_confirm_idx.store(-1, std::memory_order_relaxed);
            return 0;
        }

        if (event->mouse.button & SPC_MOUSE_BUTTON_LEFT) {
            float ax = s->delete_confirm_anchor_x;
            float ay = s->delete_confirm_anchor_y;
            auto geo = adsb_waypoint_delete::compute_geometry(ax, ay, w, h);
            float px = event->mouse.x * w;
            float py = event->mouse.y * h;

            if (adsb_waypoint_delete::contains(geo.delete_btn, px, py)) {
                int32_t idx = s->delete_confirm_idx.load(std::memory_order_relaxed);
                bool erased = false;
                {
                    std::lock_guard lock(s->waypoints_mutex);
                    // Re-validate — the host could have rewritten the list
                    // via set_list_param while the modal was up.
                    if (idx >= 0 && static_cast<size_t>(idx) < s->waypoints.size()) {
                        s->waypoints.erase(s->waypoints.begin() + idx);
                        erased = true;
                    }
                }
                if (erased) {
                    // Same cleanup as a left-click delete: drop any stale
                    // info-panel selection and notify the host so the
                    // Parameters > Map Waypoints list refreshes.
                    s->info_panel_waypoint_idx.store(-1, std::memory_order_relaxed);
                    s->host.notify_list_changed("waypoints");
                }
                s->delete_confirm_idx.store(-1, std::memory_order_relaxed);
                return 0;
            }

            // Cancel button or any click outside the modal dismisses without
            // deleting — clicks landing on the modal but not on a button
            // are swallowed.
            if (adsb_waypoint_delete::contains(geo.cancel_btn, px, py)
                || !adsb_waypoint_delete::contains(geo.modal, px, py)) {
                s->delete_confirm_idx.store(-1, std::memory_order_relaxed);
            }
            return 0;
        }
    }

    // All other events (move / wheel / release) are swallowed so they can't
    // pan, zoom, or otherwise change state while the modal is up.
    return 0;
}

int handle_event(MapDisplayState* s, const SpcInputEvent* event)
{
    // Modal short-circuit — owns all input until the user dismisses it.
    if (s->delete_confirm_idx.load(std::memory_order_acquire) >= 0) {
        return handle_delete_confirm_event(s, event);
    }

    double zoom = s->last_zoom.load(std::memory_order_relaxed);
    double origin_tx = s->last_origin_tx.load(std::memory_order_relaxed);
    double origin_ty = s->last_origin_ty.load(std::memory_order_relaxed);
    // event thread: snapshot GUI params (s->cur is worker-owned).
    const auto p = s->params.snapshot();
    int w = p.width;
    int h = p.height;

    // let overlay buttons register clicks (don't consume — map still needs the event)
    s->ui_buttons.handle_event(event, nullptr, 0, nullptr);

    // determine if cursor is over the list panel
    int map_w = s->last_map_w.load(std::memory_order_relaxed);
    float mx_px = event->mouse.x * w;
    bool over_panel = p.show_list && (mx_px >= map_w);

    if (over_panel) {
        int panel_x = map_w;
        int panel_w = w - map_w;

        uint32_t prev_selected = s->list_state.selected_icao;
        double center_lat = 0.0, center_lon = 0.0;
        int result = adsb_list::handle_list_event(
            event, panel_x, 0, panel_w, h, w, h,
            s->list_state, s->panel_aircraft,
            &center_lat, &center_lon);

        if (result == 0) {
            if (event->type == SPC_INPUT_EVENT_MOUSE_PRESS &&
                s->list_state.selected_icao != prev_selected) {
                // promote list selection to a full info-panel selection,
                // so the row click behaves like a click on the map icon
                s->info_panel_waypoint_idx.store(-1, std::memory_order_relaxed);
                s->info_panel_airport_idx.store(-1, std::memory_order_relaxed);
                s->info_panel_airspace_idx.store(-1, std::memory_order_relaxed);
                s->info_panel_icao.store(s->list_state.selected_icao,
                                          std::memory_order_relaxed);
            }
            if (event->type == SPC_INPUT_EVENT_MOUSE_PRESS && center_lat != 0.0) {
                s->center_lat.store(center_lat, std::memory_order_relaxed);
                s->center_lon.store(center_lon, std::memory_order_relaxed);
                s->interactive_view.store(true, std::memory_order_relaxed);
            }
            return 0;
        }
    }

    // ── map area event handling ───────────────────────────────────────

    switch (event->type) {
        case SPC_INPUT_EVENT_MOUSE_PRESS:
            if (event->mouse.button & SPC_MOUSE_BUTTON_LEFT) {
                s->drag_active.store(true, std::memory_order_relaxed);
                s->drag_start_x = event->mouse.x;
                s->drag_start_y = event->mouse.y;
                s->drag_start_lat = s->center_lat.load(std::memory_order_relaxed);
                s->drag_start_lon = s->center_lon.load(std::memory_order_relaxed);
                s->interactive_view.store(true, std::memory_order_relaxed);
            }
            if (event->mouse.button & SPC_MOUSE_BUTTON_RIGHT) {
                // Right-click: if there's a waypoint near the click, open the
                // delete-confirmation modal; otherwise add a new waypoint at
                // the click position. The hit-radius matches the left-click
                // info-panel feel so the gesture is forgiving.
                double click_px_x = event->mouse.x * w;
                double click_px_y = event->mouse.y * h;

                int32_t hit = hit_test_waypoint(s, zoom, origin_tx, origin_ty,
                                                static_cast<float>(click_px_x),
                                                static_cast<float>(click_px_y));
                if (hit >= 0) {
                    // Snapshot the name under the lock so the modal renders
                    // the right label even if the host rewrites the list.
                    {
                        std::lock_guard lock(s->waypoints_mutex);
                        if (static_cast<size_t>(hit) >= s->waypoints.size()) {
                            hit = -1;
                        } else {
                            std::strncpy(s->delete_confirm_name,
                                         s->waypoints[hit].name,
                                         SPC_LIST_CELL_STRING_MAX - 1);
                            s->delete_confirm_name[SPC_LIST_CELL_STRING_MAX - 1] = '\0';
                        }
                    }
                    if (hit >= 0) {
                        s->delete_confirm_anchor_x = static_cast<float>(click_px_x);
                        s->delete_confirm_anchor_y = static_cast<float>(click_px_y);
                        // Drop any in-progress drag — the modal owns input now.
                        s->drag_active.store(false, std::memory_order_relaxed);
                        // Release-store so the renderer observes the anchor /
                        // name writes above when it reads idx with acquire.
                        s->delete_confirm_idx.store(hit, std::memory_order_release);
                    }
                } else {
                    // Empty area — add a new waypoint at the click point.
                    bool mutated = false;
                    {
                        std::lock_guard lock(s->waypoints_mutex);
                        double wp_lat, wp_lon;
                        pixel_to_geo(click_px_x, click_px_y, zoom,
                                     origin_tx, origin_ty, wp_lat, wp_lon);
                        MapDisplayState::Waypoint wp{};
                        std::snprintf(wp.name, SPC_LIST_CELL_STRING_MAX,
                                      "Waypoint %d",
                                      static_cast<int>(s->waypoints.size() + 1));
                        wp.type = 0;
                        wp.lat = wp_lat;
                        wp.lon = wp_lon;
                        wp.alt = 0;
                        s->waypoints.push_back(wp);
                        mutated = true;
                    }
                    if (mutated) {
                        // Indices shifted — drop any open waypoint panel so
                        // it doesn't display stale data, and notify the host
                        // so the Parameters > Map Waypoints list refreshes.
                        s->info_panel_waypoint_idx.store(-1, std::memory_order_relaxed);
                        s->host.notify_list_changed("waypoints");
                    }
                }
            }
            return 0;

        case SPC_INPUT_EVENT_MOUSE_RELEASE:
            if (event->mouse.button & SPC_MOUSE_BUTTON_LEFT) {
                s->drag_active.store(false, std::memory_order_relaxed);

                // detect click (release near press position = no drag)
                float dx = event->mouse.x - s->drag_start_x;
                float dy = event->mouse.y - s->drag_start_y;
                float dist_px_sq = (dx * w) * (dx * w) + (dy * h) * (dy * h);
                if (dist_px_sq < 5.0f * 5.0f) {
                    float px_x = event->mouse.x * w;
                    float px_y = event->mouse.y * h;
                    uint32_t clicked_ac  = hit_test_aircraft(s, px_x, px_y);
                    int32_t  clicked_wp  = (clicked_ac != 0) ? -1
                                           : hit_test_waypoint(s, zoom, origin_tx, origin_ty,
                                                                px_x, px_y);
                    int32_t  clicked_ap  = (clicked_ac != 0 || clicked_wp >= 0)
                                           ? -1
                                           : hit_test_airport(s, px_x, px_y);
                    int32_t  clicked_as  = (clicked_ac != 0 || clicked_wp >= 0 || clicked_ap >= 0)
                                           ? -1
                                           : hit_test_airspace(s, zoom, origin_tx, origin_ty,
                                                                 px_x, px_y);
                    uint32_t current_ac = s->info_panel_icao.load(std::memory_order_relaxed);
                    int32_t  current_wp = s->info_panel_waypoint_idx.load(std::memory_order_relaxed);
                    int32_t  current_ap = s->info_panel_airport_idx.load(std::memory_order_relaxed);
                    int32_t  current_as = s->info_panel_airspace_idx.load(std::memory_order_relaxed);

                    // Precedence: aircraft > waypoint > airport > airspace.
                    // Waypoints beat reference layers because they're user-
                    // placed — clicking one is always intentional. Setting one
                    // panel always clears the others so only one shows at a time.
                    if (clicked_ac != 0) {
                        s->info_panel_waypoint_idx.store(-1, std::memory_order_relaxed);
                        s->info_panel_airport_idx.store(-1, std::memory_order_relaxed);
                        s->info_panel_airspace_idx.store(-1, std::memory_order_relaxed);
                        s->info_panel_icao.store(
                            (clicked_ac == current_ac) ? 0 : clicked_ac,
                            std::memory_order_relaxed);
                    } else if (clicked_wp >= 0) {
                        s->info_panel_icao.store(0, std::memory_order_relaxed);
                        s->info_panel_airport_idx.store(-1, std::memory_order_relaxed);
                        s->info_panel_airspace_idx.store(-1, std::memory_order_relaxed);
                        s->info_panel_waypoint_idx.store(
                            (clicked_wp == current_wp) ? -1 : clicked_wp,
                            std::memory_order_relaxed);
                    } else if (clicked_ap >= 0) {
                        s->info_panel_icao.store(0, std::memory_order_relaxed);
                        s->info_panel_waypoint_idx.store(-1, std::memory_order_relaxed);
                        s->info_panel_airspace_idx.store(-1, std::memory_order_relaxed);
                        s->info_panel_airport_idx.store(
                            (clicked_ap == current_ap) ? -1 : clicked_ap,
                            std::memory_order_relaxed);
                    } else if (clicked_as >= 0) {
                        s->info_panel_icao.store(0, std::memory_order_relaxed);
                        s->info_panel_waypoint_idx.store(-1, std::memory_order_relaxed);
                        s->info_panel_airport_idx.store(-1, std::memory_order_relaxed);
                        s->info_panel_airspace_idx.store(
                            (clicked_as == current_as) ? -1 : clicked_as,
                            std::memory_order_relaxed);
                    } else {
                        // empty click: dismiss all four
                        s->info_panel_icao.store(0, std::memory_order_relaxed);
                        s->info_panel_waypoint_idx.store(-1, std::memory_order_relaxed);
                        s->info_panel_airport_idx.store(-1, std::memory_order_relaxed);
                        s->info_panel_airspace_idx.store(-1, std::memory_order_relaxed);
                    }

                    // mirror aircraft selection into the right-side list so a
                    // map click highlights (and scrolls to) the same row a
                    // list click would have produced
                    uint32_t new_ac = s->info_panel_icao.load(std::memory_order_relaxed);
                    s->list_state.selected_icao = new_ac;
                    if (new_ac != 0) scroll_list_to_icao(s, new_ac);
                }
            }
            return 0;

        case SPC_INPUT_EVENT_MOUSE_MOVE: {
            if (s->drag_active.load(std::memory_order_relaxed)) {
                float dx_norm = event->mouse.x - s->drag_start_x;
                float dy_norm = event->mouse.y - s->drag_start_y;
                double dx_px = dx_norm * w;
                double dy_px = dy_norm * h;

                double dtx = dx_px / k_tile_size;
                double dty = dy_px / k_tile_size;

                double start_tx = lon_to_tile_x(s->drag_start_lon, zoom);
                double start_ty = lat_to_tile_y(s->drag_start_lat, zoom);

                double new_lon = tile_x_to_lon(start_tx - dtx, zoom);
                double new_lat = tile_y_to_lat(start_ty - dty, zoom);

                new_lat = std::clamp(new_lat, -85.0, 85.0);
                if (new_lon > 180.0) new_lon -= 360.0;
                if (new_lon < -180.0) new_lon += 360.0;
                s->center_lat.store(new_lat, std::memory_order_relaxed);
                s->center_lon.store(new_lon, std::memory_order_relaxed);
                // while dragging we don't care about hover labels
                s->hover_airspace_idx.store(-1, std::memory_order_relaxed);
            } else {
                // idle hover: identify which airspace the cursor is inside
                // so the render pass can draw a small label. PIP is
                // bbox-rejected per polygon, runs only over visible ones.
                float px_x = event->mouse.x * w;
                float px_y = event->mouse.y * h;
                int32_t as = hit_test_airspace(s, zoom, origin_tx, origin_ty,
                                                px_x, px_y);
                s->hover_airspace_idx.store(as, std::memory_order_relaxed);
                if (as >= 0) {
                    s->hover_px_x.store(px_x, std::memory_order_relaxed);
                    s->hover_px_y.store(px_y, std::memory_order_relaxed);
                }
            }
            return 0;
        }

        case SPC_INPUT_EVENT_MOUSE_WHEEL: {
            // zoom anchored to cursor position
            float mx = event->mouse.x;
            float my = event->mouse.y;

            double cursor_px_x = mx * w;
            double cursor_px_y = my * h;
            double cursor_lat, cursor_lon;
            pixel_to_geo(cursor_px_x, cursor_px_y, zoom,
                         origin_tx, origin_ty, cursor_lat, cursor_lon);

            double new_zoom = zoom;
            if (event->mouse.wheel_delta > 0.0f) new_zoom += 0.5;
            if (event->mouse.wheel_delta < 0.0f) new_zoom -= 0.5;
            new_zoom = std::clamp(new_zoom, 2.0, 18.0);
            if (new_zoom == zoom) return 0;

            // recompute center so the cursor position stays fixed
            double cursor_tx = lon_to_tile_x(cursor_lon, new_zoom);
            double cursor_ty = lat_to_tile_y(cursor_lat, new_zoom);

            double new_center_tx = cursor_tx - cursor_px_x / k_tile_size
                                   + static_cast<double>(map_w) / (2.0 * k_tile_size);
            double new_center_ty = cursor_ty - cursor_px_y / k_tile_size
                                   + static_cast<double>(h) / (2.0 * k_tile_size);

            double new_lon = tile_x_to_lon(new_center_tx, new_zoom);
            double new_lat = tile_y_to_lat(new_center_ty, new_zoom);
            new_lat = std::clamp(new_lat, -85.0, 85.0);
            if (new_lon > 180.0) new_lon -= 360.0;
            if (new_lon < -180.0) new_lon += 360.0;
            s->center_lon.store(new_lon, std::memory_order_relaxed);
            s->center_lat.store(new_lat, std::memory_order_relaxed);
            s->zoom.store(static_cast<float>(new_zoom), std::memory_order_relaxed);
            sync_radius_from_zoom(s, h);
            s->interactive_view.store(true, std::memory_order_relaxed);
            return 0;
        }

        default:
            return -1;
    }
}
