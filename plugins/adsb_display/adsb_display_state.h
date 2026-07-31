#pragma once

// internal shared header for the adsb_display plugin

#ifdef _WIN32
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
#endif

#include <speculor/plugin_helpers.h>
#include <cv_helpers.h>
#include <domains/adsb/schema.h>

#include "icao_country_db.h"
#include "country_flags.h"
#include "geo_helpers.h"
#include "color_palette.h"
#include "openaip_loader.h"
#include "unit_format.h"
#include <spc_ui_panel.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <spclib/http/http.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <format>
#include <fstream>
#include <memory>
#include <mutex>
#include <numbers>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <atomic>

// ── tile cache ─────────────────────────────────────────────────────────

struct TileKey
{
    int x, y, z;
    bool operator==(const TileKey& o) const { return x == o.x && y == o.y && z == o.z; }
};

struct TileKeyHash
{
    size_t operator()(const TileKey& k) const
    {
        return std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 11)
             ^ (std::hash<int>()(k.z) << 22);
    }
};

// placeholder tile: gray with grid lines
inline cv::Mat make_placeholder()
{
    cv::Mat tile(k_tile_size, k_tile_size, CV_8UC3, cv::Scalar(200, 200, 200));
    cv::rectangle(tile, {0, 0, k_tile_size, k_tile_size}, {180, 180, 180}, 1);
    return tile;
}

struct TileCache
{
    std::unordered_map<TileKey, cv::Mat, TileKeyHash> mem_cache;
    std::string base_cache_dir;
    std::string server_name;
    static constexpr size_t k_max_mem_tiles = 256;

    // Bumped on every mutation (put, evict, clear) so the background
    // cache can detect when fresh tile data has arrived asynchronously
    // and invalidate itself. Read on the process thread.
    std::atomic<uint64_t> version{0};

    void set_cache_dir(const std::string& dir)
    {
        base_cache_dir = dir;
        std::filesystem::create_directories(base_cache_dir);
    }

    void set_server(const std::string& server)
    {
        server_name = server;
        for (auto& c : server_name)
            if (c == '/' || c == ':' || c == '\\') c = '_';
    }

    const cv::Mat* get(const TileKey& key)
    {
        auto it = mem_cache.find(key);
        if (it != mem_cache.end())
            return &it->second;

        auto path = tile_path(key);
        if (std::filesystem::exists(path)) {
            auto img = cv::imread(path, cv::IMREAD_COLOR);
            if (!img.empty()) {
                cv::cvtColor(img, img, cv::COLOR_BGR2RGB);  // canvas uses RGB convention
                evict_if_full();
                auto [ins, _] = mem_cache.emplace(key, std::move(img));
                version.fetch_add(1, std::memory_order_relaxed);
                return &ins->second;
            }
        }
        return nullptr;
    }

    void put(const TileKey& key, cv::Mat tile)
    {
        auto path = tile_path(key);
        auto dir = std::filesystem::path(path).parent_path();
        std::filesystem::create_directories(dir);
        cv::imwrite(path, tile);

        evict_if_full();
        mem_cache[key] = std::move(tile);
        version.fetch_add(1, std::memory_order_relaxed);
    }

    std::string tile_path(const TileKey& key) const
    {
        return std::format("{}/{}/{}/{}/{}.png",
                          base_cache_dir, server_name, key.z, key.x, key.y);
    }

    void evict_if_full()
    {
        if (mem_cache.size() >= k_max_mem_tiles) {
            size_t to_remove = k_max_mem_tiles / 2;
            auto it = mem_cache.begin();
            while (to_remove > 0 && it != mem_cache.end()) {
                it = mem_cache.erase(it);
                --to_remove;
            }
            version.fetch_add(1, std::memory_order_relaxed);
        }
    }

    void clear() {
        mem_cache.clear();
        version.fetch_add(1, std::memory_order_relaxed);
    }
};

// ── tile downloader ────────────────────────────────────────────────────

struct TileDownloader
{
    std::unique_ptr<spclib::http::Client> http;
    std::string tile_server;
    std::string user_agent = "Speculor/0.1 (ADS-B map plugin)";

    bool init(const char* server)
    {
        tile_server = server;
        http = std::make_unique<spclib::http::Client>();
        return true;
    }

    void cleanup()
    {
        http.reset();
    }

    cv::Mat fetch(const TileKey& key)
    {
        if (!http) return {};

        auto url = std::format("https://{}/{}/{}/{}.png",
                              tile_server, key.z, key.x, key.y);

        spclib::http::RequestOptions opts;
        opts.timeout = std::chrono::seconds{10};
        opts.user_agent = user_agent;

        auto resp = http->get(url, opts);
        if (!resp.ok() || resp.body.empty())
            return {};

        auto img = cv::imdecode(resp.body, cv::IMREAD_COLOR);
        if (!img.empty())
            cv::cvtColor(img, img, cv::COLOR_BGR2RGB);  // canvas uses RGB convention
        return img;
    }
};

// ── download queue (background thread) ─────────────────────────────────

struct DownloadRequest { TileKey key; };
struct DownloadResult { TileKey key; cv::Mat tile; };

struct DownloadQueue
{
    std::mutex queue_mutex;       // protects pending + completed
    std::mutex downloader_mutex;  // protects downloader (fetch can be slow)
    std::deque<DownloadRequest> pending;
    std::deque<DownloadResult> completed;
    std::thread worker;
    std::atomic<bool> running{false};

    TileDownloader downloader;
    SpcLogContext* log_ctx = nullptr;

    void start(const char* server, SpcLogContext* log)
    {
        log_ctx = log;
        downloader.init(server);
        running.store(true);
        worker = std::thread(&DownloadQueue::thread_fn, this);
    }

    void stop()
    {
        running.store(false);
        if (worker.joinable()) worker.join();
        downloader.cleanup();
    }

    void request(const TileKey& key)
    {
        std::lock_guard lock(queue_mutex);
        for (const auto& r : pending)
            if (r.key == key) return;
        for (const auto& r : completed)
            if (r.key == key) return;
        pending.push_back({key});
    }

    void update_server(const char* server)
    {
        std::lock_guard lock(downloader_mutex);
        downloader.cleanup();
        downloader.init(server);
    }

    // drain completed downloads into cache (call from process thread only)
    void drain_completed(TileCache& cache)
    {
        std::lock_guard lock(queue_mutex);
        while (!completed.empty()) {
            auto& r = completed.front();
            cache.put(r.key, std::move(r.tile));
            completed.pop_front();
        }
    }

private:
    void thread_fn()
    {
        while (running.load(std::memory_order_relaxed)) {
            DownloadRequest req{};
            bool have_req = false;
            {
                std::lock_guard lock(queue_mutex);
                if (!pending.empty()) {
                    req = pending.front();
                    pending.pop_front();
                    have_req = true;
                }
            }

            if (!have_req) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }

            cv::Mat tile;
            {
                std::lock_guard lock(downloader_mutex);
                tile = downloader.fetch(req.key);
            }

            if (!tile.empty()) {
                std::lock_guard lock(queue_mutex);
                completed.push_back({req.key, std::move(tile)});
            }
        }
    }
};

// ── aircraft trail history ─────────────────────────────────────────────

struct TrailPoint { double lat, lon; int32_t alt; };

struct TrailHistory
{
    std::unordered_map<uint32_t, std::deque<TrailPoint>> trails;
    int32_t max_length = 30;

    void add(uint32_t icao, double lat, double lon, int32_t alt = 0)
    {
        if (lat == 0.0 && lon == 0.0) return;
        auto& trail = trails[icao];
        if (!trail.empty()) {
            auto& last = trail.back();
            if (std::abs(last.lat - lat) < 1e-6 && std::abs(last.lon - lon) < 1e-6)
                return;
        }
        trail.push_back({lat, lon, alt});
        while (static_cast<int32_t>(trail.size()) > max_length)
            trail.pop_front();
    }

    void prune(const std::vector<uint32_t>& active_icaos)
    {
        for (auto it = trails.begin(); it != trails.end(); ) {
            bool found = false;
            for (auto icao : active_icaos) {
                if (it->first == icao) { found = true; break; }
            }
            if (!found) it = trails.erase(it);
            else ++it;
        }
    }
};

// ── aircraft rendering helpers ─────────────────────────────────────────

namespace aircraft_shapes { struct Shape; }

// draw rotated airplane silhouette (uses shape data if provided, fallback otherwise)
void draw_aircraft(cv::Mat& canvas, cv::Point2d pos, float track_deg,
                   cv::Scalar color, int size,
                   const aircraft_shapes::Shape* shape,
                   cv::Scalar halo_color);

// ── aircraft view (declared before MapDisplayState for use as cached member) ──

struct AircraftView
{
    uint32_t icao;
    double lat, lon;
    int32_t alt;
    float track, gs;
    bool on_ground;
    uint8_t category = 0;
    uint8_t db_flags = 0;
    uint8_t emergency = 0;
    std::string callsign;

    // extended fields for list panel and tooltip
    uint16_t squawk = 0;
    float rssi = 0.0f;            // relative signal strength (linear)
    float seen = 0.0f;           // seconds since last message
    float seen_pos = -1.0f;      // seconds since last position (-1 = never)
    uint32_t msg_count = 0;
    float ias = 0.0f, tas = 0.0f, mach_num = 0.0f;
    float mag_heading = 0.0f, true_heading = 0.0f;
    int32_t baro_rate = 0, geom_rate = 0;
    int32_t nav_alt_mcp = 0;       // MCP-selected target altitude
    int32_t nav_alt_fms = 0;       // FMS-selected target altitude
    uint8_t msg_source = 0;        // 5 = MLAT; others = ADS-B family
    bool    spi = false;           // Special Position Identification (ident pressed)
    bool    alert = false;         // cockpit alert flag
    std::string registration;
    std::string type_code_str;
    std::string airline_icao;
    std::string origin_icao;
    std::string dest_icao;

    // enrichment data (from RECORD input — richer than TABLE fields)
    std::string origin_iata;      // "BCN"
    std::string dest_iata;        // "MUC"
    std::string origin_name;      // city name, e.g. "Barcelona"
    std::string dest_name;        // city name, e.g. "Munich"
    std::string airline_name;     // "Lufthansa"
    std::string manufacturer;     // "Boeing"
    std::string aircraft_type;    // "737-800"
    std::string owner;            // "Private"
    std::string photo_url;        // thumbnail URL

    // derived from ICAO address (pointers into static db, do not free)
    const char* country_code = nullptr;
    const char* country_name = nullptr;

    // Pre-formatted label strings — populated once per frame in
    // read_aircraft_data so the per-aircraft draw loop avoids std::format
    // and the MLAT-prefix / unit-conversion branching repeated for halo+main.
    // alt_label is empty when alt == 0 (renderer gates on that anyway).
    // vs_label is empty when the VS indicator shouldn't render this frame.
    std::string cs_label;
    std::string alt_label;
    std::string vs_label;
};

// check if aircraft has emergency status
inline bool is_emergency(const AircraftView& ac)
{
    if (ac.emergency > 0 && ac.emergency != 0xFF) return true;
    // special squawk codes (stored as octal-decoded values)
    // 7500 (hijack), 7600 (radio failure), 7700 (emergency)
    if (ac.squawk == 07500 || ac.squawk == 07600 || ac.squawk == 07700) return true;
    return false;
}

// ── aircraft screen position for hover hit-testing ────────────────────

struct AircraftScreenPos
{
    uint32_t icao;
    int px_x, px_y;
    int radius;
};

// ── airport screen position for click hit-testing ─────────────────────
// Populated each frame by the airports layer so the event handler can
// resolve clicks back to an index into MapDisplayState::reference.airports.

struct AirportScreenPos
{
    int32_t airport_idx;
    int px_x, px_y;
    int radius;
};

// ── background-cache key ─────────────────────────────────────────────
// Captures every input that can affect the static-layer rendering
// (tiles + airspace + airports + range rings + max range). When two
// frames produce identical keys, the static layers can be skipped and
// the previously-rendered map area blitted from a cached cv::Mat.
//
// All fields are POD so equality is a memberwise compare. Floats are
// quantized when they come from a continuous source (zoom, origin,
// GPS) so tiny FP noise doesn't invalidate the cache. Style/opacity
// floats come from user parameters and stay byte-stable, so they're
// stored as-is.

struct BackgroundKey
{
    // view / canvas
    int      zoom_x10           = -1;     // zoom * 10, integer for stability
    int64_t  origin_tx_x1024    = 0;      // origin_tx * 1024 (sub-pixel quantization)
    int64_t  origin_ty_x1024    = 0;
    int      map_w              = 0;
    int      h                  = 0;

    // GPS center for range rings — quantized to ~1 m
    int64_t  gps_lat_x1e7       = 0;
    int64_t  gps_lon_x1e7       = 0;
    bool     has_gps            = false;

    // selection + click highlight (drives airport accent rings)
    uint32_t selected_icao      = 0;
    int32_t  airport_click_idx  = -1;
    int32_t  show_origin_dest_highlight = 0;
    // Airline-coded fields for the selected aircraft's origin/dest
    // resolution — needed because the airport accent depends on which
    // airport pointer matches origin_icao/dest_icao. Hashing the pair
    // keeps the key compact.
    uint64_t selected_routes_hash = 0;

    // tile cache content — bumped by TileCache on every mutation
    uint64_t tile_cache_version = 0;
    bool     reference_loaded   = false;

    // layer toggles
    int32_t  show_tiles         = 0;     // composite_tiles always runs but
                                          // its output depends on map_style
    int32_t  show_airspace      = 0;
    int32_t  show_airports      = 0;
    int32_t  show_range_rings   = 0;
    int32_t  show_max_range     = 0;

    // styles + filters
    int32_t  map_style          = 0;
    int32_t  airspace_classes   = 0;
    int32_t  airspace_altitude_ft = 0;
    float    airspace_opacity   = 0.0f;
    int32_t  airport_min_type   = 0;

    // range rings
    uint32_t range_ring_color   = 0;
    int32_t  range_ring_thickness = 0;
    int32_t  range_ring_style   = 0;
    float    range_ring_opacity = 0.0f;
    float    range_ring_interval = 0.0f;
    float    range_ring_font_size = 0.0f;

    // max range
    uint32_t max_range_color    = 0;
    float    max_range_opacity  = 0.0f;
    float    max_range_font_size = 0.0f;
    int64_t  max_range_km_x100  = 0;     // max_range_km * 100 (cm precision)

    bool operator==(const BackgroundKey&) const = default;
};

// ── list panel interactive state ──────────────────────────────────────

struct ListPanelState
{
    int sort_column = 0;       // 0=ICAO, 1=callsign, 2=country, 3=type, 4=status, 5=alt, 6=speed, 7=seen, 8=tracked
    bool sort_ascending = true;
    int scroll_offset = 0;
    uint32_t selected_icao = 0;
    uint32_t hovered_row_icao = 0;
};

// ── plugin state ───────────────────────────────────────────────────────

struct MapDisplayState
{
    spc::HostServices host;

    cv::Mat canvas;
    SpcFrame output_frame{};

    // Sticky background cache. Holds the last-rendered map_rect contents
    // (tiles + airspace + airports + range rings + max range). When the
    // current frame's BackgroundKey equals bg_key and bg_cache is valid,
    // the static-layer calls are skipped and bg_cache is blitted into
    // canvas(map_rect) instead. Captures back into bg_cache on cache miss.
    cv::Mat bg_cache;
    BackgroundKey bg_key;
    bool bg_key_valid = false;

    TileCache tile_cache;
    DownloadQueue download_queue;
    TrailHistory trail_history;

    // ── GUI-settable parameters (H6 thread-safety) ─────────────────────
    // set_parameter (GUI thread) writes these under the SharedParams mutex;
    // process() (worker) takes one snapshot per frame into `cur` and every
    // layer reads from `cur`. Reconfiguring worker-owned stateful objects
    // (download_queue / tile_cache / trail_history) is deferred to the worker
    // via params_dirty. Excludes the viewport fields (atomics below) and any
    // worker-tracked / internal state (max_range_km, offsets, caches, ...).
    struct Params
    {
        int32_t width = 1200;
        int32_t height = 800;
        int32_t map_style = MAP_STYLE_DARK;
        char custom_server[SPC_PARAM_STRING_MAX] = "tile.openstreetmap.org";
        int32_t show_labels = 1;
        int32_t show_trails = 1;
        int32_t trail_length = 30;
        float icon_scale = 1.0f;
        float font_size = 0.35f;
        int32_t unit_system = 1;  // 0=Imperial, 1=Metric
        int32_t show_trend_vector = 1;
        float trend_seconds = 15.0f;
        int32_t show_vs_indicator = 1;
        int32_t show_staleness_fade = 1;

        // filters (stored in imperial units regardless of unit_system)
        int32_t filter_alt_min = 0;
        int32_t filter_alt_max = 0;              // 0 = unlimited
        float   filter_distance_max_km = 0.0f;   // 0 = unlimited
        int32_t filter_show_ground = 1;
        int32_t filter_military_only = 0;
        char    filter_callsign_substr[SPC_PARAM_STRING_MAX] = "";

        // airspace overlay
        int32_t show_airspace = 0;
        int32_t airspace_classes = 3;     // 0=None 1=Controlled 2=Restricted 3=All
        float   airspace_opacity = 0.10f;
        int32_t airspace_altitude_ft = 0;

        // range rings
        int32_t show_range_rings = 1;
        float range_ring_interval = 20.0f;  // km
        uint32_t range_ring_color = 0x00FF00FF;  // RGBA green
        int32_t range_ring_thickness = 1;
        int32_t range_ring_style = 0;      // 0=solid, 1=dashed, 2=dotted
        float range_ring_opacity = 0.15f;
        float range_ring_font_size = 0.30f;

        // max range styling (the tracked distance max_range_km is worker-owned)
        int32_t show_max_range = 1;
        uint32_t max_range_color = 0xFF6600FF;  // RGBA orange
        float max_range_opacity = 0.25f;
        float max_range_font_size = 0.30f;

        // info panel
        float info_font_size = 0.30f;
        float info_panel_opacity = 0.92f;

        // aircraft list panel
        int32_t show_list = 1;
        int32_t list_width = 220;

        // airports overlay
        int32_t show_airports = 1;
        int32_t airport_min_type = 1;       // 0=Large 1=Medium+ 2=All
        int32_t show_origin_dest_highlight = 1;
    };

    spc::SharedParams<Params> params;
    // worker-owned snapshot, refreshed once at the top of process()
    Params cur;
    // set by set_parameter when a param needs worker-side reconfiguration
    // (custom_server / map_style / trail_length); cleared in process()
    std::atomic<bool> params_dirty{false};

    // last tile-server config actually applied on the worker, so a params_dirty
    // raised by an unrelated param (e.g. trail_length) doesn't needlessly evict
    // the tile cache. Seeded in start(), compared in process().
    int32_t applied_map_style = MAP_STYLE_DARK;
    char applied_custom_server[SPC_PARAM_STRING_MAX] = "";

    // cached input field offsets (aircraft table)
    uint32_t off_icao = 0, off_callsign = 0;
    uint32_t off_alt = 0, off_gs = 0, off_track = 0;
    uint32_t off_lat = 0, off_lon = 0;
    uint32_t off_ground = 0;
    uint32_t off_category = 0, off_db_flags = 0, off_emergency = 0;
    uint32_t off_squawk = 0, off_rssi = 0, off_seen = 0, off_seen_pos = 0;
    uint32_t off_msg_count = 0;
    uint32_t off_ias = 0, off_tas = 0, off_mach = 0;
    uint32_t off_mag_heading = 0, off_true_heading = 0;
    uint32_t off_baro_rate = 0, off_geom_rate = 0;
    uint32_t off_nav_alt_mcp = 0, off_nav_alt_fms = 0;
    uint32_t off_msg_source = 0, off_spi = 0, off_alert = 0;
    uint32_t off_registration = 0, off_type_code = 0;
    uint32_t off_airline_icao = 0, off_origin_icao = 0, off_dest_icao = 0;
    bool offsets_resolved = false;

    // cached input field offsets (gps table)
    uint32_t off_gps_lat = 0, off_gps_lon = 0;
    bool gps_offsets_resolved = false;

    // cached GPS position (persists across frames when GPS data is sparse)
    double cached_gps_lat = 0.0;
    double cached_gps_lon = 0.0;
    bool has_cached_gps = false;

    // cached aircraft data (persists across frames when input is sparse)
    std::vector<AircraftView> cached_aircraft;

    // snapshot of the sorted+filtered aircraft list as last rendered into
    // the right-side list panel. The click hit-test must index into this
    // exact vector — cached_aircraft is unfiltered/unsorted, so using it
    // would map row N to the wrong plane whenever a filter is active or
    // a non-default sort is selected.
    std::vector<AircraftView> panel_aircraft;

    // ── viewport (bidirectional: set_parameter + on_input_event + process) ───
    // Atomic because the GUI thread, the event thread, and the worker all
    // read/write these. Cross-field tearing during a drag is cosmetic and
    // pre-existing — no mutex (matches the existing interactive_view etc).
    std::atomic<int32_t> view_mode{0};
    std::atomic<double> center_lat{51.5};
    std::atomic<double> center_lon{-0.1};
    std::atomic<float> zoom{6.0f};
    std::atomic<float> radius{50.0f};

    // tracked maximum distance — worker-owned (written in process)
    double max_range_km = 0.0;

    uint64_t frame_number = 0;

    // when true, compute_view uses center_lat/lon/zoom directly
    // (set by on_input_event, cleared when user changes view_mode via parameter panel)
    std::atomic<bool> interactive_view{false};

    // interactive state (updated by on_input_event, read by process)
    std::atomic<bool> drag_active{false};
    float drag_start_x = 0.0f, drag_start_y = 0.0f;
    double drag_start_lat = 0.0, drag_start_lon = 0.0;

    // last computed view (written by process, read by on_input_event for coord conversion)
    std::atomic<float> last_zoom{6.0f};
    std::atomic<double> last_origin_tx{0.0};
    std::atomic<double> last_origin_ty{0.0};
    std::atomic<int> last_map_w{1200};

    // ── aircraft list panel ───────────────────────────────────────────
    ListPanelState list_state;

    // ── animation counter ──────────────────────────────────────────────
    uint32_t frame_number_local = 0;

    // ── info panel ────────────────────────────────────────────────────
    std::atomic<uint32_t> info_panel_icao{0};  // 0 = hidden

    // screen positions of rendered aircraft (double-buffered for thread safety)
    // process() writes to build buffer, then swaps; event handler reads active buffer
    std::vector<AircraftScreenPos> screen_pos[2];
    std::atomic<int> screen_pos_active{0};  // index into screen_pos[] for event handler

    // airport screen positions + which airport's info panel is showing.
    // info_panel_airport_idx == -1 means no airport panel. The index is into
    // reference.airports, which is immutable after start() so this stays stable.
    std::vector<AirportScreenPos> airport_screen_pos[2];
    std::atomic<int> airport_screen_pos_active{0};
    std::atomic<int32_t> info_panel_airport_idx{-1};

    // Which airspace polygons were actually drawn this frame (indices into
    // reference.airspaces). Published by airspace_layer, read by the click
    // hit-test so only visible polygons participate. Double-buffered like
    // aircraft / airport positions.
    std::vector<int32_t> visible_airspace_idx[2];
    std::atomic<int> visible_airspace_active{0};
    std::atomic<int32_t> info_panel_airspace_idx{-1};

    // Hover state for the airspace "quick label" overlay: updated on
    // MOUSE_MOVE via point-in-polygon over visible airspaces; drawn at the
    // cursor position on the next frame. -1 means no airspace under cursor.
    std::atomic<int32_t> hover_airspace_idx{-1};
    std::atomic<float>   hover_px_x{0.0f};
    std::atomic<float>   hover_px_y{0.0f};

    // ── first-seen tracking (for "time tracked" column) ───────────────
    std::unordered_map<uint32_t, std::chrono::steady_clock::time_point> first_seen;

    // ── country flag cache ────────────────────────────────────────────
    CountryFlagCache flag_cache;

    // ── enrichment data (parsed from RECORD input) ───────────────────
    struct EnrichmentRecord {
        std::string registration;
        std::string aircraft_type;   // "737-800"
        std::string icao_type;       // "B738"
        std::string manufacturer;
        std::string owner;
        std::string country;
        std::string photo_url;
        std::string photo_thumb_url;
        std::string airline_name;
        std::string airline_icao;
        std::string origin_icao;
        std::string origin_iata;
        std::string origin_name;
        std::string origin_municipality;
        double origin_lat = 0, origin_lon = 0;
        std::string dest_icao;
        std::string dest_iata;
        std::string dest_name;
        std::string dest_municipality;
        double dest_lat = 0, dest_lon = 0;
    };
    std::unordered_map<uint32_t, EnrichmentRecord> enrichment_cache;

    // Snapshot of the last RECORD JSON we successfully parsed. The upstream
    // Aircraft Enricher emits the same enrichment blob every tick — at
    // 23 aircraft the JSON is ~50 KB and nlohmann::json::parse + the
    // ~15 string allocations per entry burn ~13 ms of CPU per frame
    // (70%+ of the plugin's per-frame budget). Skip the parse entirely
    // when the incoming JSON byte-matches what we parsed last frame; the
    // resulting enrichment_cache writes would all be no-ops.
    std::string last_enrichment_json;

    // ── photo thumbnail download + cache ─────────────────────────────
    struct PhotoRequest { uint32_t icao; std::string url; };
    struct PhotoResult  { uint32_t icao; cv::Mat thumbnail; };

    struct PhotoDownloadQueue {
        std::mutex mutex;
        std::deque<PhotoRequest> pending;
        std::deque<PhotoResult> completed;
        std::unordered_set<uint32_t> requested;  // all ICAOs ever queued
        std::thread worker;
        std::atomic<bool> running{false};
        std::unique_ptr<spclib::http::Client> http;
        SpcLogContext* log = nullptr;
        std::string cache_dir;

        void start(SpcLogContext* log_ctx) {
            log = log_ctx;
            // photo cache directory
            auto home = std::filesystem::path(
#ifdef _WIN32
                std::getenv("USERPROFILE") ? std::getenv("USERPROFILE") : "."
#else
                std::getenv("HOME") ? std::getenv("HOME") : "."
#endif
            );
            cache_dir = (home / ".speculor" / "photos").string();
            std::filesystem::create_directories(cache_dir);

            http = std::make_unique<spclib::http::Client>();
            running.store(true);
            worker = std::thread(&PhotoDownloadQueue::thread_fn, this);
        }

        void stop() {
            running.store(false);
            if (worker.joinable()) worker.join();
            http.reset();
        }

        void request(uint32_t icao, const std::string& url) {
            if (url.empty()) return;
            std::lock_guard lock(mutex);
            if (requested.contains(icao)) return;
            requested.insert(icao);
            pending.push_back({icao, url});
        }

        void drain_completed(std::unordered_map<uint32_t, cv::Mat>& cache) {
            std::lock_guard lock(mutex);
            while (!completed.empty()) {
                auto& r = completed.front();
                cache[r.icao] = std::move(r.thumbnail);
                completed.pop_front();
            }
        }

    private:
        static constexpr int THUMB_W = 80;
        static constexpr int THUMB_H = 60;

        void thread_fn() {
            while (running.load(std::memory_order_relaxed)) {
                PhotoRequest req{};
                bool have = false;
                {
                    std::lock_guard lock(mutex);
                    if (!pending.empty()) {
                        req = std::move(pending.front());
                        pending.pop_front();
                        have = true;
                    }
                }
                if (!have) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }

                // check disk cache first
                char hex[7];
                icao_to_hex(req.icao, hex);
                auto path = cache_dir + "/" + hex + ".jpg";
                if (std::filesystem::exists(path)) {
                    auto img = cv::imread(path, cv::IMREAD_COLOR);
                    if (!img.empty()) {
                        cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
                        if (img.cols != THUMB_W || img.rows != THUMB_H)
                            cv::resize(img, img, {THUMB_W, THUMB_H}, 0, 0, cv::INTER_AREA);
                        std::lock_guard lock(mutex);
                        completed.push_back({req.icao, std::move(img)});
                        continue;
                    }
                }

                // download
                spclib::http::RequestOptions opts;
                opts.timeout = std::chrono::seconds{15};
                opts.user_agent = "Speculor/1.0";

                auto resp = http->get(req.url, opts);
                if (!resp.ok()) continue;

                // decode + resize
                auto img = cv::imdecode(resp.body, cv::IMREAD_COLOR);
                if (img.empty()) continue;

                cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

                // save original to disk cache
                cv::Mat bgr;
                cv::cvtColor(img, bgr, cv::COLOR_RGB2BGR);
                cv::imwrite(path, bgr);

                // resize for display
                if (img.cols != THUMB_W || img.rows != THUMB_H)
                    cv::resize(img, img, {THUMB_W, THUMB_H}, 0, 0, cv::INTER_AREA);

                std::lock_guard lock(mutex);
                completed.push_back({req.icao, std::move(img)});

                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        }
    };

    PhotoDownloadQueue photo_queue;
    std::unordered_map<uint32_t, cv::Mat> photo_cache;   // ICAO → thumbnail

    // ── map overlay buttons ───────────────────────────────────────────
    spc::ui::Panel ui_buttons{spc::ui::Theme{}};

    // ── waypoints (list parameter) ───────────────────────────────────
    struct Waypoint {
        char name[SPC_LIST_CELL_STRING_MAX]{};
        int32_t type = 0;  // 0=Waypoint, 1=Airport, 2=Military Base, 3=VOR, 4=NDB
        double lat = 0, lon = 0, alt = 0;
    };
    std::vector<Waypoint> waypoints;
    std::mutex waypoints_mutex;

    // info_panel_waypoint_idx == -1 means no waypoint panel. Indexes into
    // `waypoints`; cleared on any mutation of the vector since adds / removes
    // shift indices.
    std::atomic<int32_t> info_panel_waypoint_idx{-1};

    // Right-click delete confirmation modal. delete_confirm_idx == -1 means
    // the modal isn't active. While it is, the event handler short-circuits
    // every other map interaction (drag, zoom, list panel, overlay buttons)
    // — the user must explicitly Cancel or Delete first.
    //
    // Anchor + name are written *before* the atomic store so a renderer
    // observing idx >= 0 (acquire) sees the matching popup data.
    std::atomic<int32_t> delete_confirm_idx{-1};
    float                delete_confirm_anchor_x = 0.0f;
    float                delete_confirm_anchor_y = 0.0f;
    char                 delete_confirm_name[SPC_LIST_CELL_STRING_MAX]{};

    // ── reference data (airspace + airports, loaded once at start) ──
    // Loaded asynchronously on a background thread — parsing ~340 MB of
    // JSON takes 10-30 s and would otherwise freeze the UI. Layers must
    // check `reference_loaded.load(acquire)` before reading `reference` or
    // `airport_by_icao`; the loader thread flips the flag with release
    // semantics only after both containers are fully populated.
    ReferenceData reference;
    std::unordered_map<std::string, const AirportFeature*> airport_by_icao;
    std::atomic<bool> reference_loaded{false};
    std::jthread loader_thread;   // joins on plugin-state destruction
};

