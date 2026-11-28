#include "adsb_display_state.h"
#include "aircraft_list_panel.h"
#include "aircraft_tooltip.h"
#include "aircraft_layer.h"
#include "airport_tooltip.h"
#include "airports_layer.h"
#include "airspace_layer.h"
#include "airspace_tooltip.h"
#include "filters.h"
#include "input_parser.h"
#include "interactions.h"
#include "map_layers.h"
#include "map_tiles.h"
#include "plugin_data_dir.h"
#include "view_math.h"
#include "waypoint_delete_confirm.h"
#include "waypoint_tooltip.h"

#include <speculor/table_helpers.h>
#include <spc_clock.h>
#ifdef SPC_HAS_VULKAN
#include <gpu/gpu_host.h>
#endif

#include <cstring>

SPC_PLUGIN_CAST(MapDisplayState)
SPC_PLUGIN_HOST_SERVICES(MapDisplayState, host)

// convenience alias for the GUI-settable parameter block (H6)
using Params = MapDisplayState::Params;

// ── descriptor ─────────────────────────────────────────────────────────

SPC_PLUGIN_DESCRIPTOR(
    spc::DescriptorBuilder("adsb_display", "ADS-B Display", "ADS-B/Visualization")
        .author("Speculor").version("0.2.0")
        .help("Renders ADS-B aircraft positions on an OpenStreetMap tile map")
        .maturity(SPC_MATURITY_STABLE)
        .tags({"ads-b", "tracking"})
        SPC_IF_VULKAN(.gpu_compute())
        // field order is mirrored by AircraftField / GpsField in input_parser.cpp
        .input_table("aircraft_in", "Aircraft", {
            {"icao_addr",       SPC_FIELD_UINT32},
            {"callsign",        SPC_FIELD_STRING32},
            {"alt_baro",        SPC_FIELD_INT32},
            {"alt_geom",        SPC_FIELD_INT32},
            {"ground_speed",    SPC_FIELD_FLOAT},
            {"ias",             SPC_FIELD_FLOAT},
            {"tas",             SPC_FIELD_FLOAT},
            {"mach",            SPC_FIELD_FLOAT},
            {"track",           SPC_FIELD_FLOAT},
            {"track_rate",      SPC_FIELD_FLOAT},
            {"mag_heading",     SPC_FIELD_FLOAT},
            {"true_heading",    SPC_FIELD_FLOAT},
            {"roll",            SPC_FIELD_FLOAT},
            {"baro_rate",       SPC_FIELD_INT32},
            {"geom_rate",       SPC_FIELD_INT32},
            {"latitude",        SPC_FIELD_FLOAT64},
            {"longitude",       SPC_FIELD_FLOAT64},
            {"nav_alt_mcp",     SPC_FIELD_INT32},
            {"nav_alt_fms",     SPC_FIELD_INT32},
            {"nav_heading",     SPC_FIELD_FLOAT},
            {"nav_qnh",         SPC_FIELD_FLOAT},
            {"squawk",          SPC_FIELD_UINT16},
            {"category",        SPC_FIELD_UINT8},
            {"emergency",       SPC_FIELD_UINT8},
            {"spi",             SPC_FIELD_BOOL},
            {"alert",           SPC_FIELD_BOOL},
            {"is_on_ground",    SPC_FIELD_BOOL},
            {"nic",             SPC_FIELD_UINT8},
            {"nac_p",           SPC_FIELD_UINT8},
            {"nac_v",           SPC_FIELD_UINT8},
            {"sil",             SPC_FIELD_UINT8},
            {"nic_baro",        SPC_FIELD_UINT8},
            {"gva",             SPC_FIELD_UINT8},
            {"sda",             SPC_FIELD_UINT8},
            {"db_flags",        SPC_FIELD_UINT8},
            {"msg_source",      SPC_FIELD_UINT8},
            {"adsb_version",    SPC_FIELD_UINT8},
            {"rssi",            SPC_FIELD_FLOAT},
            {"msg_count",       SPC_FIELD_UINT32},
            {"seen",            SPC_FIELD_FLOAT},
            {"seen_pos",        SPC_FIELD_FLOAT},
            {"rc",              SPC_FIELD_UINT32},
            {"registration",    SPC_FIELD_STRING32},
            {"type_code",       SPC_FIELD_STRING32},
            {"airline_icao",    SPC_FIELD_STRING32},
            {"origin_icao",     SPC_FIELD_STRING32},
            {"dest_icao",       SPC_FIELD_STRING32},
        }, 4, SPC_CONSUME_NON_BLOCKING)
        .input("enrichment_in", "Enrichment", SPC_DATA_JSON, 1, SPC_CONSUME_NON_BLOCKING)
        .input_table("gps_in", "GPS Position", {
            {"latitude",  SPC_FIELD_FLOAT64},
            {"longitude", SPC_FIELD_FLOAT64},
        }, 1, SPC_CONSUME_NON_BLOCKING)
        .output("map_out", "Map", SPC_DATA_FRAME)
        .enum_param("view_mode", "View Mode", {"Auto", "Manual"}, 0, "View")
            .param_help("Auto centers on all aircraft, Manual uses fixed coordinates")
        .float64_param("center_lat", "Center Latitude", -90.0, 90.0, 51.5, 0.001, "View")
            .param_help("Map center latitude in Manual view mode")
        .float64_param("center_lon", "Center Longitude", -180.0, 180.0, -0.1, 0.001, "View")
            .param_help("Map center longitude in Manual view mode")
        .float_param("zoom", "Zoom", 2.0f, 18.0f, 6.0f, 0.1f, "View")
            .param_help("Map zoom level (higher = more detail, smaller area)")
        .float_param("radius", "Radius (km)", 1.0f, 5000.0f, 50.0f, 1.0f, "View")
            .param_help("Display radius in kilometers for range ring calculation")
        .int_param("width", "Width", 320, 3840, 1200, 10, "View")
            .param_help("Output frame width in pixels")
        .int_param("height", "Height", 240, 2160, 800, 10, "View")
            .param_help("Output frame height in pixels")
        .enum_param("map_style", "Map Style",
                    {"Standard", "Humanitarian", "Dark", "Light", "Topo", "Custom"},
                    2, "Tiles")
            .param_help("Map tile source and visual style")
        .string_param("custom_server", "Custom Server", "tile.openstreetmap.org", "Tiles")
            .param_help("Custom tile server hostname (e.g. tile.openstreetmap.org)")
        .float_param("icon_scale", "Icon Scale", 0.3f, 5.0f, 1.0f, 0.1f, "Display")
            .param_help("Aircraft icon size multiplier")
        .float_param("font_size", "Font Size", 0.1f, 2.0f, 0.35f, 0.05f, "Display")
            .param_help("Aircraft label font size multiplier")
        .enum_param("font", "Font", {"Inter", "Noto Sans", "Roboto"}, 0, "Display")
            .param_help("Typeface of every label, panel and tooltip")
        .enum_param("unit_system", "Units", {"Imperial", "Metric"}, 1, "Display")
            .param_help("Unit system: Imperial (ft, kts, fpm) or Metric (m, km/h, m/min)")
        .bool_param("show_labels", "Show Labels", true, "Display")
            .param_help("Show callsign and altitude labels on aircraft")
        .bool_param("show_trails", "Show Trails", true, "Display")
            .param_help("Draw position history trail behind each aircraft")
        .int_param("trail_length", "Trail Length", 5, 300, 30, 5, "Display")
            .param_help("Number of trail points to keep per aircraft")
        .bool_param("show_trend_vector", "Show Trend Vector", true, "Display")
            .param_help("Line ahead of each aircraft projecting its position forward")
        .float_param("trend_seconds", "Trend Seconds", 10.0f, 180.0f, 15.0f, 5.0f, "Display")
            .param_help("How many seconds ahead the trend-vector line projects")
        .bool_param("show_vs_indicator", "Show Vertical Speed", true, "Display")
            .param_help("Climb/descent arrow + rate next to the altitude label")
        .bool_param("show_staleness_fade", "Fade Stale Tracks", true, "Display")
            .param_help("Fade aircraft whose data is going stale before they expire")
        .int_param("filter_alt_min", "Min Altitude (ft)", 0, 60000, 0, 100, "Filters")
            .param_help("Hide airborne aircraft below this altitude (feet, independent of unit system)")
        .int_param("filter_alt_max", "Max Altitude (ft)", 0, 60000, 0, 100, "Filters")
            .param_help("Hide airborne aircraft above this altitude (feet, 0 = unlimited)")
        .float_param("filter_distance_max_km", "Max Distance (km)", 0.0f, 5000.0f, 0.0f, 1.0f, "Filters")
            .param_help("Hide aircraft farther than this from the receiver (0 = unlimited)")
        .bool_param("filter_show_ground", "Show Ground Traffic", true, "Filters")
            .param_help("Include aircraft reporting on-ground status")
        .bool_param("filter_military_only", "Military Only", false, "Filters")
            .param_help("Show only aircraft flagged as military in the ICAO DB")
        .string_param("filter_callsign_substr", "Callsign Contains", "", "Filters")
            .param_help("Show only aircraft whose callsign contains this text (case-insensitive; empty = all)")
        .bool_param("show_airports", "Show Airports", true, "Airports")
            .param_help("Plot airport markers from the fetched reference dataset")
        .enum_param("airport_min_type", "Airport Detail",
                    {"Large only", "Large + Medium", "All"}, 1, "Airports")
            .param_help("How many airport size tiers to draw (small airports and heliports also gate on zoom)")
        .bool_param("show_origin_dest_highlight", "Highlight Origin/Dest", true, "Airports")
            .param_help("For the selected aircraft, accent its origin and destination airports and draw a line to the destination")
        .bool_param("show_airspace", "Show Airspace", false, "Airspace")
            .param_help("Overlay airspace polygons (FIR/CTR/TMA/P-R-D/MATZ) if data was fetched at build time")
        .enum_param("airspace_classes", "Airspace Classes",
                    {"None", "Controlled", "Restricted", "All"}, 3, "Airspace")
            .param_help("Which airspace categories to show. Controlled = FIR/CTR/TMA etc; Restricted = P/R/D/MATZ.")
        .float_param("airspace_opacity", "Airspace Opacity", 0.0f, 1.0f, 0.10f, 0.05f, "Airspace")
            .param_help("Fill opacity for airspace polygons (outlines always full opacity)")
        .int_param("airspace_altitude_ft", "Altitude Filter (ft)", 0, 60000, 0, 500, "Airspace")
            .param_help("Show only airspaces whose floor/ceiling bracket this altitude. 0 = no filter.")
        .bool_param("show_range_rings", "Show Range Rings", true, "Range Rings")
            .param_help("Draw concentric distance rings on the map")
        .float_param("range_ring_interval", "Interval (km)", 1.0f, 100.0f, 20.0f, 1.0f, "Range Rings")
            .param_help("Distance between range rings in kilometers")
        .color_param("range_ring_color", "Ring Color", 0x00FF00FF, "Range Rings")
            .param_help("Range ring line color")
        .int_param("range_ring_thickness", "Ring Thickness", 1, 5, 1, 1, "Range Rings")
            .param_help("Range ring line thickness in pixels")
        .enum_param("range_ring_style", "Ring Style", {"Solid", "Dashed", "Dotted"}, 0, "Range Rings")
            .param_help("Range ring line style")
        .float_param("range_ring_opacity", "Ring Opacity", 0.0f, 1.0f, 0.15f, 0.05f, "Range Rings")
            .param_help("Range ring transparency (0 = invisible, 1 = opaque)")
        .float_param("range_ring_font_size", "Ring Font Size", 0.1f, 2.0f, 0.30f, 0.05f, "Range Rings")
        // Max Range group
        .bool_param("show_max_range", "Show Max Range", true, "Max Range")
            .param_help("Display a circle at the maximum distance any aircraft has been seen")
        .color_param("max_range_color", "Color", 0xFF6600FF, "Max Range")
        .float_param("max_range_opacity", "Opacity", 0.0f, 1.0f, 0.25f, 0.05f, "Max Range")
        .float_param("max_range_font_size", "Font Size", 0.1f, 2.0f, 0.30f, 0.05f, "Max Range")
            .param_help("Font size for range ring distance labels")
        .float_param("info_font_size", "Font Size", 0.2f, 0.6f, 0.30f, 0.02f, "Info Panel")
            .param_help("Font size for the aircraft info panel")
        .float_param("info_panel_opacity", "Panel Opacity", 0.0f, 1.0f, 0.92f, 0.05f, "Info Panel")
            .param_help("Opacity of the aircraft info panel background (0 = transparent, 1 = opaque)")
        .bool_param("show_list", "Show Aircraft List", true, "Aircraft List")
            .param_help("Show scrollable aircraft list panel on the right side")
        .int_param("list_width", "List Width", 150, 500, 220, 10, "Aircraft List")
            .param_help("Width of the aircraft list panel in pixels")
        .list_param("waypoints", "Waypoints", "Map Waypoints")
            .param_help("Points of interest drawn on the map (right-click map to add)")
            .list_column("name", "Name", SPC_PARAM_STRING)
            .list_column_enum("type", "Type", {"Waypoint", "Airport", "Military Base", "VOR", "NDB"})
            .list_column("lat", "Latitude", SPC_PARAM_FLOAT64, -90.0, 90.0, 0.0001)
            .list_column("lon", "Longitude", SPC_PARAM_FLOAT64, -180.0, 180.0, 0.0001)
            .list_column("alt", "Altitude (ft)", SPC_PARAM_FLOAT64, 0.0, 100000.0, 1.0)
)

// ── lifecycle ──────────────────────────────────────────────────────────

static SpcPluginInstance* create_instance()
{
    auto* s = new MapDisplayState{};

    std::string home;
#ifdef _WIN32
    if (auto* appdata = std::getenv("LOCALAPPDATA"))
        home = appdata;
    else
        home = ".";
    s->tile_cache.set_cache_dir(home + "/speculor/tiles");
#else
    if (auto* h = std::getenv("HOME"))
        home = h;
    else
        home = "/tmp";
    s->tile_cache.set_cache_dir(home + "/.speculor/tiles");
#endif

    return reinterpret_cast<SpcPluginInstance*>(s);
}

static void destroy_instance(SpcPluginInstance* inst)
{
    auto* s = state(inst);
    s->download_queue.stop();
    // Join the reference-data loader explicitly — the thread touches
    // members of `s`, so we need it finished before we free `s`.
    // jthread's destructor would join for us, but only after earlier
    // members had already started destructing; this order is safer.
    if (s->loader_thread.joinable()) s->loader_thread.join();
    delete s;
}


// ── parameters ─────────────────────────────────────────────────────────

// The user pans and zooms on the map itself, and zoom and radius each rewrite
// the other: give the host what the view has become, so it is shown and saved.
static void push_view(MapDisplayState* s)
{
    static const uint32_t k_lat = spc::param_index(info(), "center_lat");
    static const uint32_t k_lon = spc::param_index(info(), "center_lon");
    static const uint32_t k_zoom = spc::param_index(info(), "zoom");
    static const uint32_t k_radius = spc::param_index(info(), "radius");

    const double lat = s->center_lat.load(std::memory_order_relaxed);
    const double lon = s->center_lon.load(std::memory_order_relaxed);
    const float zoom = s->zoom.load(std::memory_order_relaxed);
    const float radius = s->radius.load(std::memory_order_relaxed);
    if (lat != s->pushed_lat) { s->host.set_float64(k_lat, lat); s->pushed_lat = lat; }
    if (lon != s->pushed_lon) { s->host.set_float64(k_lon, lon); s->pushed_lon = lon; }
    if (zoom != s->pushed_zoom) { s->host.set_float(k_zoom, zoom); s->pushed_zoom = zoom; }
    if (radius != s->pushed_radius) { s->host.set_float(k_radius, radius); s->pushed_radius = radius; }
}

static int set_parameter(SpcPluginInstance* inst, const char* name,
                         const SpcParameterDesc* value)
{
    auto* s = state(inst);

    // ── viewport fields (atomics, bidirectional) ───────────────────────
    // Kept out of the Params block because the event thread and the worker
    // also mutate these. set_parameter clears interactive_view so a panel
    // edit overrides any in-progress pan/zoom.
    if (std::strcmp(name, "view_mode") == 0 && value->type == SPC_PARAM_ENUM) {
        s->view_mode.store(value->enum_val.value, std::memory_order_relaxed);
        s->interactive_view.store(false, std::memory_order_relaxed);
        return 0;
    }
    if (std::strcmp(name, "center_lat") == 0 && value->type == SPC_PARAM_FLOAT64) {
        s->center_lat.store(value->float64_val.value, std::memory_order_relaxed);
        s->interactive_view.store(false, std::memory_order_relaxed);
        return 0;
    }
    if (std::strcmp(name, "center_lon") == 0 && value->type == SPC_PARAM_FLOAT64) {
        s->center_lon.store(value->float64_val.value, std::memory_order_relaxed);
        s->interactive_view.store(false, std::memory_order_relaxed);
        return 0;
    }
    if (std::strcmp(name, "zoom") == 0 && value->type == SPC_PARAM_FLOAT) {
        s->zoom.store(value->float_val.value, std::memory_order_relaxed);
        s->interactive_view.store(false, std::memory_order_relaxed);
        const Params p = s->params.snapshot();
        sync_radius_from_zoom(s, p.height);
        push_view(s);
        return 0;
    }
    if (std::strcmp(name, "radius") == 0 && value->type == SPC_PARAM_FLOAT) {
        s->radius.store(value->float_val.value, std::memory_order_relaxed);
        s->interactive_view.store(false, std::memory_order_relaxed);
        const Params p = s->params.snapshot();
        sync_zoom_from_radius(s, p.width, p.height);
        push_view(s);
        return 0;
    }

    // ── GUI-only params (SharedParams) ─────────────────────────────────
    bool changed = s->params.update([&](Params& p) {
        return spc::try_set_int(name, value, "width", p.width)
            || spc::try_set_int(name, value, "height", p.height)
            || spc::try_set_enum(name, value, "map_style", p.map_style)
            || spc::try_set_string(name, value, "custom_server", p.custom_server)
            || spc::try_set_float(name, value, "icon_scale", p.icon_scale)
            || spc::try_set_float(name, value, "font_size", p.font_size)
            || spc::try_set_enum(name, value, "font", p.font)
            || spc::try_set_enum(name, value, "unit_system", p.unit_system)
            || spc::try_set_bool(name, value, "show_labels", p.show_labels)
            || spc::try_set_bool(name, value, "show_trails", p.show_trails)
            || spc::try_set_int(name, value, "trail_length", p.trail_length)
            || spc::try_set_bool(name, value, "show_trend_vector", p.show_trend_vector)
            || spc::try_set_float(name, value, "trend_seconds", p.trend_seconds)
            || spc::try_set_bool(name, value, "show_vs_indicator", p.show_vs_indicator)
            || spc::try_set_bool(name, value, "show_staleness_fade", p.show_staleness_fade)
            || spc::try_set_int(name, value, "filter_alt_min", p.filter_alt_min)
            || spc::try_set_int(name, value, "filter_alt_max", p.filter_alt_max)
            || spc::try_set_float(name, value, "filter_distance_max_km", p.filter_distance_max_km)
            || spc::try_set_bool(name, value, "filter_show_ground", p.filter_show_ground)
            || spc::try_set_bool(name, value, "filter_military_only", p.filter_military_only)
            || spc::try_set_string(name, value, "filter_callsign_substr", p.filter_callsign_substr)
            || spc::try_set_bool(name, value, "show_airspace", p.show_airspace)
            || spc::try_set_enum(name, value, "airspace_classes", p.airspace_classes)
            || spc::try_set_float(name, value, "airspace_opacity", p.airspace_opacity)
            || spc::try_set_int(name, value, "airspace_altitude_ft", p.airspace_altitude_ft)
            || spc::try_set_bool(name, value, "show_airports", p.show_airports)
            || spc::try_set_enum(name, value, "airport_min_type", p.airport_min_type)
            || spc::try_set_bool(name, value, "show_origin_dest_highlight", p.show_origin_dest_highlight)
            || spc::try_set_bool(name, value, "show_range_rings", p.show_range_rings)
            || spc::try_set_float(name, value, "range_ring_interval", p.range_ring_interval)
            || spc::try_set_color(name, value, "range_ring_color", p.range_ring_color)
            || spc::try_set_int(name, value, "range_ring_thickness", p.range_ring_thickness)
            || spc::try_set_enum(name, value, "range_ring_style", p.range_ring_style)
            || spc::try_set_float(name, value, "range_ring_opacity", p.range_ring_opacity)
            || spc::try_set_float(name, value, "range_ring_font_size", p.range_ring_font_size)
            || spc::try_set_bool(name, value, "show_max_range", p.show_max_range)
            || spc::try_set_color(name, value, "max_range_color", p.max_range_color)
            || spc::try_set_float(name, value, "max_range_opacity", p.max_range_opacity)
            || spc::try_set_float(name, value, "max_range_font_size", p.max_range_font_size)
            || spc::try_set_float(name, value, "info_font_size", p.info_font_size)
            || spc::try_set_float(name, value, "info_panel_opacity", p.info_panel_opacity)
            || spc::try_set_bool(name, value, "show_list", p.show_list)
            || spc::try_set_int(name, value, "list_width", p.list_width);
    });
    if (!changed) return -1;

    // Defer worker-owned reconfiguration (download_queue / tile_cache /
    // trail_history) to process() — never touch those objects from the GUI
    // thread. map_style / custom_server change the tile server; trail_length
    // changes the trail cap.
    if (std::strcmp(name, "map_style") == 0
        || std::strcmp(name, "custom_server") == 0
        || std::strcmp(name, "trail_length") == 0) {
        s->params_dirty.store(true, std::memory_order_release);
    }

    return 0;
}

static void param_rules(const SpcValue* values, uint32_t* states)
{
    spc::ParamRules r(info(), values, states);

    const bool auto_view = r.choice("view_mode") == 0;
    r.disable("center_lat", auto_view);
    r.disable("center_lon", auto_view);
    r.disable("zoom", auto_view);
    r.disable("radius", auto_view);

    r.disable("custom_server", r.choice("map_style") != MAP_STYLE_CUSTOM);
    r.disable("trail_length", !r.on("show_trails"));
    r.disable("trend_seconds", !r.on("show_trend_vector"));

    const bool airspace = r.on("show_airspace");
    r.disable("airspace_classes", !airspace);
    r.disable("airspace_opacity", !airspace);
    r.disable("airspace_altitude_ft", !airspace);

    const bool airports = r.on("show_airports");
    r.disable("airport_min_type", !airports);
    r.disable("show_origin_dest_highlight", !airports);

    const bool rings = r.on("show_range_rings");
    r.disable("range_ring_interval", !rings);
    r.disable("range_ring_color", !rings);
    r.disable("range_ring_thickness", !rings);
    r.disable("range_ring_style", !rings);
    r.disable("range_ring_opacity", !rings);
    r.disable("range_ring_font_size", !rings);

    const bool max_range = r.on("show_max_range");
    r.disable("max_range_color", !max_range);
    r.disable("max_range_opacity", !max_range);
    r.disable("max_range_font_size", !max_range);

    r.disable("list_width", !r.on("show_list"));
}

// ── list parameter handlers ────────────────────────────────────────────

void push_waypoints(MapDisplayState* s)
{
    const SpcNodeInfo* desc = info();
    const uint32_t param = spc::param_index(desc, "waypoints");
    std::vector<SpcListRow> rows;
    {
        std::lock_guard lock(s->waypoints_mutex);
        rows.resize(std::min<size_t>(s->waypoints.size(), SPC_LIST_ROW_MAX));
        for (size_t i = 0; i < rows.size(); ++i) {
            std::strncpy(rows[i].cells[0].string_val, s->waypoints[i].name, SPC_LIST_CELL_STRING_MAX - 1);
            rows[i].cells[1].enum_val = s->waypoints[i].type;
            rows[i].cells[2].float64_val = s->waypoints[i].lat;
            rows[i].cells[3].float64_val = s->waypoints[i].lon;
            rows[i].cells[4].float64_val = s->waypoints[i].alt;
        }
    }
    SpcListSchema schema;
    spc::list_schema_of(desc->params[param], schema);
    s->host.set_list(param, schema, rows.data(), static_cast<int32_t>(rows.size()));
}

static int set_list_rows_impl(MapDisplayState* s, const char* name,
                                const SpcListRow* rows, int32_t count)
{
    if (std::strcmp(name, "waypoints") != 0) return -1;
    {
        std::lock_guard lock(s->waypoints_mutex);
        s->waypoints.resize(count);
        for (int32_t i = 0; i < count; ++i) {
            std::strncpy(s->waypoints[i].name, rows[i].cells[0].string_val, SPC_LIST_CELL_STRING_MAX - 1);
            s->waypoints[i].name[SPC_LIST_CELL_STRING_MAX - 1] = '\0';
            s->waypoints[i].type = rows[i].cells[1].enum_val;
            s->waypoints[i].lat = rows[i].cells[2].float64_val;
            s->waypoints[i].lon = rows[i].cells[3].float64_val;
            s->waypoints[i].alt = rows[i].cells[4].float64_val;
        }
    }
    // Indices may have shifted — drop any open waypoint panel so it doesn't
    // display stale data.
    s->info_panel_waypoint_idx.store(-1, std::memory_order_relaxed);
    return 0;
}

SPC_PLUGIN_LIST_HANDLER(MapDisplayState, set_list_rows_impl)

// A new instance, or a loaded project, is told zoom and radius together, each
// undoing the other as it is applied. Zoom is the one the map is drawn from, so
// it is the one kept, and the radius follows from it at the frame size that
// arrived in the same call.
static void params_changed(SpcPluginInstance* inst, const SpcValue* values, uint64_t changed)
{
    static const uint32_t k_zoom = spc::param_index(info(), "zoom");
    static const uint32_t k_radius = spc::param_index(info(), "radius");

    spc::params_by_name<info, set_parameter, set_list_rows>(inst, values, changed);

    const uint64_t both = (uint64_t{1} << k_zoom) | (uint64_t{1} << k_radius);
    if ((changed & both) != both) return;
    auto* s = state(inst);
    s->zoom.store(values[k_zoom].f, std::memory_order_relaxed);
    sync_radius_from_zoom(s, s->params.snapshot().height);
    push_view(s);
}

// ── streaming ──────────────────────────────────────────────────────────

static int start(SpcPluginInstance* inst)
{
    auto* s = state(inst);
    const Params p = s->params.snapshot();
    s->sequence = 0;
    s->trail_history.trails.clear();
    s->trail_history.max_length = p.trail_length;
    s->cached_aircraft.clear();

    // initialize aircraft list and tooltip state
    s->flag_cache.load();
    s->first_seen.clear();
    s->list_state = {};
    s->info_panel_icao.store(0, std::memory_order_relaxed);
    s->info_panel_airport_idx.store(-1, std::memory_order_relaxed);
    s->info_panel_airspace_idx.store(-1, std::memory_order_relaxed);
    s->screen_pos[0].clear();
    s->screen_pos[1].clear();
    s->screen_pos_active.store(0, std::memory_order_relaxed);
    s->airport_screen_pos[0].clear();
    s->airport_screen_pos[1].clear();
    s->airport_screen_pos_active.store(0, std::memory_order_relaxed);
    s->visible_airspace_idx[0].clear();
    s->visible_airspace_idx[1].clear();
    s->visible_airspace_active.store(0, std::memory_order_relaxed);

    auto* server = resolve_tile_server(p.map_style, p.custom_server);
    s->tile_cache.set_server(server);
    s->tile_cache.set_style(p.map_style);
    s->applied_map_style = p.map_style;
    std::strncpy(s->applied_custom_server, p.custom_server, sizeof(s->applied_custom_server) - 1);
    s->applied_custom_server[sizeof(s->applied_custom_server) - 1] = '\0';
    s->download_queue.start(server, &s->host.cached_log);
    s->photo_queue.start(&s->host.cached_log);
    s->photo_cache.clear();
    s->enrichment_cache.clear();
    s->max_range_km = 0.0;
    s->bg_key_valid = false;
    s->bg_image_stale = true;
    // clear any pending param reconfiguration — start applied current params
    s->params_dirty.store(false, std::memory_order_release);
#ifdef SPC_HAS_VULKAN
    s->vk = spc::gpu::context_from_host(s->host.svc);
    s->gpu_ready = s->vk && s->gpu_drawer.init(*s->vk);
    if (!s->gpu_ready) s->vk.reset();
#endif

    // Loaded on a thread: a first-run parse takes tens of seconds, which start()
    // must not hold the engine for. Readers gate on `reference_loaded` (acquire).
    if (!s->reference_loaded.load(std::memory_order_acquire)
        && !s->loader_thread.joinable()) {
        auto data_dir = spc::plugin_data::own_module_dir() / "data" / "openaip";
        auto airports_path = (data_dir / "airports.json").string();
        auto airspace_path = (data_dir / "airspace.json").string();

        // destroy_instance joins it, so interrupt and stop ask it to end early
        s->loader_thread = std::jthread([s, airports_path, airspace_path](std::stop_token stop) {
            auto t0 = std::chrono::steady_clock::now();
            s->reference = load_reference_data(airports_path, airspace_path, stop);
            if (stop.stop_requested()) return;
            // Build the ICAO -> airport pointer map before flipping the
            // ready flag so readers never see a half-initialized map.
            s->airport_by_icao.reserve(s->reference.airports.size());
            for (const auto& a : s->reference.airports) {
                if (!a.ident.empty()) s->airport_by_icao.emplace(a.ident, &a);
            }
            auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0).count();
            // release ensures the writes above are visible to any thread
            // that subsequently does an acquire-load on the flag.
            s->reference_loaded.store(true, std::memory_order_release);
            SPC_LOG_INFO(&s->host.cached_log,
                         "adsb display reference data ready: %zu airports, %zu airspaces (%lld ms)",
                         s->reference.airports.size(),
                         s->reference.airspaces.size(),
                         static_cast<long long>(dt));
        });
    }

    SPC_LOG_INFO(&s->host.cached_log, "adsb display started (tiles from %s, zoom %.1f)",
                 server, static_cast<double>(s->zoom.load(std::memory_order_relaxed)));
    return 0;
}

static void request_stop(SpcPluginInstance* inst)
{
    auto* s = state(inst);
    s->download_queue.interrupt();
    s->photo_queue.interrupt();
    s->loader_thread.request_stop();
}

static int stop(SpcPluginInstance* inst)
{
    auto* s = state(inst);
    s->loader_thread.request_stop();
    s->download_queue.stop();
    s->photo_queue.stop();
    s->flag_cache.clear();
    s->first_seen.clear();
    s->screen_pos[0].clear();
    s->screen_pos[1].clear();
    s->list.clear();
    s->static_list.clear();
#ifdef SPC_HAS_VULKAN
    if (s->vk) {
        s->gpu_drawer.destroy(*s->vk);
        s->vk.reset();
    }
    s->gpu_ready = false;
#endif
    SPC_LOG_INFO(&s->host.cached_log, "adsb display stopped (%llu frames)",
                 static_cast<unsigned long long>(s->sequence));
    return 0;
}

SPC_PLUGIN_UI_INPUT(MapDisplayState, handle_event)

// ── process ────────────────────────────────────────────────────────────


// compute center lat/lon and zoom level from view mode, GPS input, and aircraft
static void compute_view(const MapDisplayState* s,
                         const std::vector<AircraftView>& aircraft,
                         bool has_gps, double gps_lat, double gps_lon,
                         int w, int h,
                         double& clat, double& clon, double& zoom)
{
    clat = s->center_lat.load(std::memory_order_relaxed);
    clon = s->center_lon.load(std::memory_order_relaxed);
    zoom = s->zoom.load(std::memory_order_relaxed);

    // interactive pan/zoom: use center_lat/lon/zoom directly
    if (s->interactive_view.load(std::memory_order_relaxed))
        return;

    // determine center: GPS overrides manual center when available
    if (has_gps) {
        clat = gps_lat;
        clon = gps_lon;
    }

    if (s->view_mode.load(std::memory_order_relaxed) == 1) {
        // manual mode: use zoom parameter directly
    } else {
        // auto mode: center on GPS (or manual center), zoom to fit all aircraft
        if (!aircraft.empty()) {
            double min_lat = 90, max_lat = -90, min_lon = 180, max_lon = -180;
            for (const auto& ac : aircraft) {
                min_lat = std::min(min_lat, ac.lat);
                max_lat = std::max(max_lat, ac.lat);
                min_lon = std::min(min_lon, ac.lon);
                max_lon = std::max(max_lon, ac.lon);
            }

            // if no GPS, also center on aircraft
            if (!has_gps) {
                clat = (min_lat + max_lat) / 2.0;
                clon = (min_lon + max_lon) / 2.0;
            }

            if (aircraft.size() > 1) {
                // when centered on GPS, span must include all aircraft relative to center
                double span_lat, span_lon;
                if (has_gps) {
                    // ensure span covers from center to furthest aircraft in every direction
                    double far_north = std::max(max_lat - clat, clat - min_lat);
                    double far_east  = std::max(max_lon - clon, clon - min_lon);
                    span_lat = far_north * 2.0 * 1.1;
                    span_lon = far_east  * 2.0 * 1.1;
                } else {
                    span_lat = (max_lat - min_lat) * 1.1;
                    span_lon = (max_lon - min_lon) * 1.1;
                }
                zoom = zoom_for_span(span_lat, span_lon, w, h);
            } else {
                // single aircraft: use radius parameter for zoom
                double lat_span, lon_span;
                radius_to_span(s->radius.load(std::memory_order_relaxed), clat, lat_span, lon_span);
                zoom = zoom_for_span(lat_span, lon_span, w, h);
            }
        } else {
            // no aircraft yet: use radius parameter for initial zoom
            double lat_span, lon_span;
            radius_to_span(s->radius.load(std::memory_order_relaxed), clat, lat_span, lon_span);
            zoom = zoom_for_span(lat_span, lon_span, w, h);
        }
    }
}
// Build the BackgroundKey from current rendering inputs. Quantizes
// continuous floats so trivial FP noise doesn't invalidate the cache,
// hashes the selected aircraft's origin/dest ICAOs (drives the airport
// accent rings) into a single uint64_t.
static BackgroundKey compute_bg_key(MapDisplayState* s,
                                    const MapDisplayState::Params& cur,
                                    double zoom,
                                    double origin_tx, double origin_ty,
                                    int map_w, int h,
                                    double gps_lat, double gps_lon,
                                    bool has_gps)
{
    BackgroundKey k;
    k.zoom_x10        = static_cast<int>(std::round(zoom * 10.0));
    k.origin_tx_x1024 = static_cast<int64_t>(std::round(origin_tx * 1024.0));
    k.origin_ty_x1024 = static_cast<int64_t>(std::round(origin_ty * 1024.0));
    k.map_w           = map_w;
    k.h               = h;

    // Coarsen GPS to ~10m so noisy GPS sources don't bust the cache every
    // frame. Range rings shift at most a pixel or two between buckets at
    // typical zooms — invisible.
    k.gps_lat_x1e7    = static_cast<int64_t>(std::round(gps_lat * 1e5));
    k.gps_lon_x1e7    = static_cast<int64_t>(std::round(gps_lon * 1e5));
    k.has_gps         = has_gps;

    k.selected_icao   = s->info_panel_icao.load(std::memory_order_relaxed);
    k.airport_click_idx = s->info_panel_airport_idx.load(std::memory_order_relaxed);
    k.show_origin_dest_highlight = cur.show_origin_dest_highlight;

    // Hash selected aircraft's origin/dest ICAOs — captures the airport
    // accent ring state without copying the strings into the key.
    uint64_t routes_hash = 0;
    if (k.selected_icao != 0 && k.show_origin_dest_highlight) {
        for (const auto& ac : s->cached_aircraft) {
            if (ac.icao != k.selected_icao) continue;
            std::hash<std::string> sh;
            routes_hash = sh(ac.origin_icao) ^ (sh(ac.dest_icao) * 0x9e3779b97f4a7c15ULL);
            break;
        }
    }
    k.selected_routes_hash = routes_hash;

    k.tile_cache_version = s->tile_cache.version.load(std::memory_order_relaxed);
    k.reference_loaded   = s->reference_loaded.load(std::memory_order_acquire);

    k.show_tiles         = 1;  // always renders; map_style differentiates output
    k.show_airspace      = cur.show_airspace;
    k.show_airports      = cur.show_airports;
    k.show_range_rings   = cur.show_range_rings;
    k.show_max_range     = cur.show_max_range;

    k.map_style          = cur.map_style;
    k.font               = cur.font;
    k.airspace_classes   = cur.airspace_classes;
    k.airspace_altitude_ft = cur.airspace_altitude_ft;
    k.airspace_opacity   = cur.airspace_opacity;
    k.airport_min_type   = cur.airport_min_type;

    k.range_ring_color   = cur.range_ring_color;
    k.range_ring_thickness = cur.range_ring_thickness;
    k.range_ring_style   = cur.range_ring_style;
    k.range_ring_opacity = cur.range_ring_opacity;
    k.range_ring_interval = cur.range_ring_interval;
    k.range_ring_font_size = cur.range_ring_font_size;

    k.max_range_color    = cur.max_range_color;
    k.max_range_opacity  = cur.max_range_opacity;
    k.max_range_font_size = cur.max_range_font_size;
    // 1 km bucket — max_range_km grows as the farthest aircraft moves out,
    // and at cm precision a single jet at the edge of range bumped the key
    // every frame. The visible MAX ring is at km granularity anyway.
    k.max_range_km_x100  = static_cast<int64_t>(std::round(s->max_range_km));

    return k;
}

// What the frame shows, into s->list. The map area's static layers are in
// s->bg_rgb, drawn again only when their key changes; the GPU path draws them
// from s->bg_image as the list's first command, the CPU path copies them in.
static void build_frame(MapDisplayState* s, const SpcStep* step, bool for_gpu)
{
    const SpcData* inputs = step->inputs;
    const uint32_t input_count = step->input_count;

    // ── per-frame parameter snapshot (H6) ──────────────────────────────
    // exchange BEFORE snapshot so an edit landing in the window stays flagged
    // for next frame. Take exactly one snapshot into s->cur; every layer call
    // below reads from s->cur, so the frame sees a single consistent set.
    const bool params_changed = s->params_dirty.exchange(false, std::memory_order_acquire);
    s->cur = s->params.snapshot();

    // dirty-apply worker-owned stateful objects (never touched from the GUI
    // thread): tile server (download_queue + tile_cache) and trail length.
    if (params_changed) {
        // trail length cap is cheap and unrelated to the tile server — always apply
        s->trail_history.max_length = s->cur.trail_length;
        // only rebuild the tile server + evict the cache when it actually changed,
        // so editing trail_length (which shares params_dirty) doesn't wipe the tiles
        if (s->cur.map_style != s->applied_map_style
            || std::strcmp(s->cur.custom_server, s->applied_custom_server) != 0) {
            auto* server = resolve_tile_server(s->cur.map_style, s->cur.custom_server);
            s->download_queue.update_server(server);
            s->tile_cache.set_server(server);
            s->tile_cache.set_style(s->cur.map_style);
            s->tile_cache.clear();
            s->applied_map_style = s->cur.map_style;
            std::strncpy(s->applied_custom_server, s->cur.custom_server, sizeof(s->applied_custom_server) - 1);
            s->applied_custom_server[sizeof(s->applied_custom_server) - 1] = '\0';
        }
    }

    int w = s->cur.width;
    int h = s->cur.height;

    // compute map area width (shrink when list panel is shown)
    int map_w = w;
    int panel_w = 0;
    if (s->cur.show_list) {
        panel_w = std::clamp(s->cur.list_width, 200, w / 2);
        map_w = w - panel_w;
    }
    s->last_map_w.store(map_w, std::memory_order_relaxed);

    // parse enrichment metadata from RECORD input (port 1)
    parse_enrichment_record(s, inputs, input_count);

    auto fresh = read_aircraft_data(s, inputs, input_count);
    bool new_aircraft_data = !fresh.empty();
    if (new_aircraft_data)
        s->cached_aircraft = std::move(fresh);
    auto& aircraft = s->cached_aircraft;

    // update first-seen tracking
    auto now = std::chrono::steady_clock::now();
    for (const auto& ac : aircraft) {
        if (s->first_seen.find(ac.icao) == s->first_seen.end())
            s->first_seen[ac.icao] = now;
    }
    // prune stale entries (aircraft gone for > 5 minutes)
    for (auto it = s->first_seen.begin(); it != s->first_seen.end(); ) {
        bool active = false;
        for (const auto& ac : aircraft) {
            if (ac.icao == it->first) { active = true; break; }
        }
        if (!active && (now - it->second) > std::chrono::minutes(5))
            it = s->first_seen.erase(it);
        else
            ++it;
    }

    double gps_lat = 0.0, gps_lon = 0.0;
    if (read_gps_position(inputs, input_count, gps_lat, gps_lon)) {
        s->cached_gps_lat = gps_lat;
        s->cached_gps_lon = gps_lon;
        s->has_cached_gps = true;
    }
    bool has_gps = s->has_cached_gps;
    gps_lat = s->cached_gps_lat;
    gps_lon = s->cached_gps_lon;

    double clat, clon, zoom;
    compute_view(s, aircraft, has_gps, gps_lat, gps_lon, map_w, h, clat, clon, zoom);

    // compute visible tile range (using map_w, not full w)
    double center_tx = lon_to_tile_x(clon, zoom);
    double center_ty = lat_to_tile_y(clat, zoom);

    double origin_tx = center_tx - static_cast<double>(map_w) / (2.0 * k_tile_size);
    double origin_ty = center_ty - static_cast<double>(h) / (2.0 * k_tile_size);

    // publish view state for the event handler
    s->last_zoom.store(static_cast<float>(zoom), std::memory_order_relaxed);
    s->last_origin_tx.store(origin_tx, std::memory_order_relaxed);
    s->last_origin_ty.store(origin_ty, std::memory_order_relaxed);

    // drain completed downloads into cache (thread-safe handoff)
    s->download_queue.drain_completed(s->tile_cache);
    s->photo_queue.drain_completed(s->photo_cache);

    // queue photo downloads + track max range
    for (const auto& ac : aircraft) {
        // photo: queue download if we have a URL but no cached thumbnail
        if (!ac.photo_url.empty() && !s->photo_cache.contains(ac.icao))
            s->photo_queue.request(ac.icao, ac.photo_url);

        // max range: update if GPS is available
        if (has_gps && ac.lat != 0.0 && ac.lon != 0.0) {
            double dist = haversine_km(gps_lat, gps_lon, ac.lat, ac.lon);
            if (dist > s->max_range_km)
                s->max_range_km = dist;
        }
    }

    // ── sticky background ──────────────────────────────────────────────
    // The static layers (tiles + airspace + airports + range rings + max
    // range) depend only on view + style + selection state — none of which
    // change frame-to-frame on a stable view, so they are drawn again only
    // when the BackgroundKey changes.
    BackgroundKey cur_key = compute_bg_key(s, s->cur, zoom, origin_tx, origin_ty,
                                           map_w, h, gps_lat, gps_lon, has_gps);

    bool cache_hit = s->bg_key_valid &&
                     cur_key == s->bg_key &&
                     s->bg_rgb.rows == h &&
                     s->bg_rgb.cols == map_w;

    if (!cache_hit) {
        if (s->bg_rgb.rows != h || s->bg_rgb.cols != map_w) s->bg_rgb = cv::Mat(h, map_w, CV_8UC3);
        s->bg_rgb.setTo(cv::Scalar(200, 200, 200));
        composite_tiles(s, s->bg_rgb, zoom, origin_tx, origin_ty, map_w, h);
        s->static_list.clear();
        // airspace sits between tiles and aircraft so polygons color the map
        // without drowning out icons drawn later
        render_airspace_layer(s, s->static_list, zoom, origin_tx, origin_ty, map_w, h);
        // airport markers render above airspace but below aircraft so they
        // read as landmarks rather than as traffic
        render_airports_layer(s, s->static_list, zoom, origin_tx, origin_ty, map_w, h);
        if (has_gps) {
            render_range_rings(s, s->static_list, gps_lat, gps_lon, zoom, origin_tx, origin_ty, map_w, h);
            render_max_range(s, s->static_list, gps_lat, gps_lon, zoom, origin_tx, origin_ty, map_w, h);
        }
        s->cpu_drawer.draw(s->bg_rgb.data, static_cast<uint32_t>(map_w), static_cast<uint32_t>(h),
                           static_cast<uint32_t>(s->bg_rgb.step[0]), SPC_PIXEL_FORMAT_RGB24, s->static_list);
        s->bg_key = cur_key;
        s->bg_key_valid = true;
        s->bg_image_stale = true;
    }

    auto& list = s->list;
    list.clear();
    if (for_gpu) {
        if (s->bg_image_stale) {
            s->bg_image.assign(s->bg_rgb.data, static_cast<uint32_t>(map_w), static_cast<uint32_t>(h),
                               static_cast<uint32_t>(s->bg_rgb.step[0]), SPC_PIXEL_FORMAT_RGB24);
            s->bg_image_stale = false;
        }
        adsb_draw::image_at(list, s->bg_image, 0, 0);
    }
    const int32_t font = s->cur.font;

    // render waypoints on map
    {
        std::lock_guard lock(s->waypoints_mutex);
        for (const auto& wp : s->waypoints) {
            double tx = lon_to_tile_x(wp.lon, zoom) - origin_tx;
            double ty = lat_to_tile_y(wp.lat, zoom) - origin_ty;
            int px = static_cast<int>(tx * k_tile_size);
            int py = static_cast<int>(ty * k_tile_size);
            if (px < -20 || px > map_w + 20 || py < -20 || py > h + 20) continue;

            cv::Scalar color;
            adsb_draw::Marker marker_type = adsb_draw::Marker::Diamond;
            switch (wp.type) {
                case 0: color = map_contrast_color(s->cur.map_style); marker_type = adsb_draw::Marker::Diamond; break;  // waypoint: adaptive diamond
                case 1: color = {50, 180, 255};  marker_type = adsb_draw::Marker::Square;  break;  // airport: blue square
                case 2: color = {255, 60, 60};   marker_type = adsb_draw::Marker::Star;    break;  // military: red star
                case 3: color = {50, 200, 50};   marker_type = adsb_draw::Marker::TiltedCross; break; // VOR: green cross
                case 4: color = {255, 180, 50};  marker_type = adsb_draw::Marker::TriangleUp;  break; // NDB: orange triangle
                default: color = {200, 200, 200}; break;
            }

            adsb_draw::marker(list, adsb_draw::px(px, py), marker_type, 12, 2, adsb_draw::rgb(color));

            // draw name label with background
            if (wp.name[0] != '\0') {
                const auto st = adsb_draw::style(font, 0.35, color);
                const int tw = static_cast<int>(std::lround(adsb_draw::text_width(wp.name, st)));
                const int th = static_cast<int>(std::lround(adsb_draw::text_height(st)));
                int lx = px + 8, ly = py - 4;
                auto wp_label_bg = is_dark_map(s->cur.map_style)
                    ? cv::Scalar(0, 0, 0) : cv::Scalar(240, 240, 240);
                adsb_draw::fill_px(list, lx - 1, ly - th - 1, lx + tw + 1, ly + 2, adsb_draw::rgb(wp_label_bg));
                adsb_draw::text(list, lx, ly, wp.name, st);
            }
        }
    }

    // update trail state only when fresh aircraft data arrived this frame.
    // Trails track *all* observed aircraft — filtering only affects rendering,
    // so toggling filters doesn't wipe history.
    if (new_aircraft_data && s->cur.show_trails) {
        std::vector<uint32_t> active_icaos;
        active_icaos.reserve(aircraft.size());
        for (const auto& ac : aircraft) {
            active_icaos.push_back(ac.icao);
            s->trail_history.add(ac.icao, ac.lat, ac.lon, ac.alt);
        }
        s->trail_history.prune(active_icaos);
    }

    // Apply filters once, in a single place, so the map and the list panel
    // see the exact same subset (selection / hit-testing stay consistent).
    std::vector<AircraftView> filtered;
    filtered.reserve(aircraft.size());
    for (const auto& ac : aircraft) {
        if (passes_filter(ac, s->cur, gps_lat, gps_lon, has_gps))
            filtered.push_back(ac);
    }

    render_aircraft_layer(s, list, filtered, zoom, origin_tx, origin_ty, map_w, h);

    // dest-airport leader line — drawn live (not cached) because it
    // depends on the selected aircraft's current pixel position, which
    // moves between frames. Originally lived inside render_airports_layer
    // but moved out so the airports layer can be cached.
    if (s->cur.show_origin_dest_highlight) {
        uint32_t sel_icao = s->info_panel_icao.load(std::memory_order_relaxed);
        if (sel_icao != 0 && s->reference_loaded.load(std::memory_order_acquire)) {
            for (const auto& ac : filtered) {
                if (ac.icao != sel_icao) continue;
                auto ait = s->airport_by_icao.find(ac.dest_icao);
                if (ait == s->airport_by_icao.end()) break;
                const auto* dest_ap = ait->second;
                if (!dest_ap) break;
                auto ac_px = geo_to_pixel(ac.lat, ac.lon, zoom, origin_tx, origin_ty);
                auto dest_px = geo_to_pixel(dest_ap->lat, dest_ap->lon,
                                            zoom, origin_tx, origin_ty);
                const auto p0 = adsb_draw::px(static_cast<int>(ac_px.x), static_cast<int>(ac_px.y));
                const auto p1 = adsb_draw::px(static_cast<int>(dest_px.x), static_cast<int>(dest_px.y));
                if (std::hypot(p1.x - p0.x, p1.y - p0.y) >= 5.0f)
                    adsb_draw::dashed_line(list, p0, p1, 8, 5, 1, adsb_draw::Color{255, 200, 40});
                break;
            }
        }
    }

    // distance/bearing line from GPS to selected aircraft
    {
        uint32_t sel_icao = s->info_panel_icao.load(std::memory_order_relaxed);
        if (sel_icao != 0 && has_gps) {
            for (const auto& ac : filtered) {
                if (ac.icao != sel_icao) continue;
                auto gps_px = geo_to_pixel(gps_lat, gps_lon, zoom, origin_tx, origin_ty);
                auto ac_px = geo_to_pixel(ac.lat, ac.lon, zoom, origin_tx, origin_ty);

                // dashed line from GPS to aircraft
                const int x0 = static_cast<int>(gps_px.x), y0 = static_cast<int>(gps_px.y);
                const int x1 = static_cast<int>(ac_px.x), y1 = static_cast<int>(ac_px.y);
                if (std::hypot(x1 - x0, y1 - y0) > 5.0) {
                    const auto contrast = map_contrast_color(s->cur.map_style);
                    adsb_draw::dashed_line(list, adsb_draw::px(x0, y0), adsb_draw::px(x1, y1), 8, 5, 1,
                                           adsb_draw::rgb(contrast));

                    // distance and bearing label at midpoint
                    double dist_km = haversine_km(gps_lat, gps_lon, ac.lat, ac.lon);
                    double brg = bearing_deg(gps_lat, gps_lon, ac.lat, ac.lon);
                    std::string dist_label;
                    if (dist_km < 1.0)
                        dist_label = std::format("{:.0f}m {:.0f}deg", dist_km * 1000.0, brg);
                    else
                        dist_label = std::format("{:.1f}km {:.0f}deg", dist_km, brg);

                    int mx = (x0 + x1) / 2;
                    int my = (y0 + y1) / 2;
                    const auto st = adsb_draw::style(font, 0.35, contrast);
                    const int tw = static_cast<int>(std::lround(adsb_draw::text_width(dist_label, st)));
                    const int th = static_cast<int>(std::lround(adsb_draw::text_height(st)));
                    auto dist_bg = is_dark_map(s->cur.map_style)
                        ? cv::Scalar(0x2e, 0x1e, 0x1e)
                        : cv::Scalar(0xf0, 0xf0, 0xf0);
                    adsb_draw::fill_px(list, mx - tw / 2 - 2, my - th - 2, mx + tw / 2 + 2, my + 3,
                                       adsb_draw::rgb(dist_bg));
                    adsb_draw::text(list, mx - tw / 2, my, dist_label, st);
                }
                break;
            }
        }
    }

    // map overlay buttons (top-left)
    {
        static const adsb_draw::Theme th;
        const auto st = adsb_draw::style(font, adsb_draw::kButtonFont, th.text);
        const char* labels[2] = {"Center GPS", "Fit All"};
        cv::Rect rects[2];
        int bx = adsb_draw::kMargin;
        for (int i = 0; i < 2; ++i) {
            const float tw = adsb_draw::text_width(labels[i], st);
            const int bw = std::max(static_cast<int>(std::lround(tw)) + 14, 36);
            const int by = adsb_draw::kMargin, bh = adsb_draw::kButtonHeight;
            rects[i] = {bx, by, bw, bh};
            adsb_draw::fill_px(list, bx, by, bx + bw - 1, by + bh - 1, adsb_draw::rgb(th.surface1));
            adsb_draw::stroke_px(list, bx, by, bx + bw - 1, by + bh - 1, adsb_draw::rgb(th.surface2));
            adsb_draw::text_vcenter(list, bx + (bw - tw) / 2, by, bh, labels[i], st);
            bx += bw + adsb_draw::kButtonGap;
        }
        s->map_buttons.publish(rects[0], rects[1], w, h);
        bool gps_clicked = s->map_buttons.take_click(0);
        bool fit_clicked = s->map_buttons.take_click(1);

        if (gps_clicked && has_gps) {
            s->center_lat.store(gps_lat, std::memory_order_relaxed);
            s->center_lon.store(gps_lon, std::memory_order_relaxed);
            s->interactive_view.store(true, std::memory_order_relaxed);
        }
        if (fit_clicked && !filtered.empty()) {
            double mn_lat = 90, mx_lat = -90, mn_lon = 180, mx_lon = -180;
            for (const auto& ac : filtered) {
                mn_lat = std::min(mn_lat, ac.lat);
                mx_lat = std::max(mx_lat, ac.lat);
                mn_lon = std::min(mn_lon, ac.lon);
                mx_lon = std::max(mx_lon, ac.lon);
            }
            double lat_margin = (mx_lat - mn_lat) * 0.10;
            double lon_margin = (mx_lon - mn_lon) * 0.10;
            mn_lat -= lat_margin; mx_lat += lat_margin;
            mn_lon -= lon_margin; mx_lon += lon_margin;
            s->center_lat.store((mn_lat + mx_lat) / 2.0, std::memory_order_relaxed);
            s->center_lon.store((mn_lon + mx_lon) / 2.0, std::memory_order_relaxed);
            s->zoom.store(static_cast<float>(zoom_for_span(
                mx_lat - mn_lat, mx_lon - mn_lon, map_w, h)),
                std::memory_order_relaxed);
            sync_radius_from_zoom(s, h);
            s->interactive_view.store(true, std::memory_order_relaxed);
        }
    }

    // compass rose (top-right of map area)
    render_compass_rose(s, list, map_w, h);

    // status bar (map area only). Show filtered/total count when a filter is
    // hiding anything so the user isn't surprised by a missing aircraft.
    {
        std::string status;
        if (filtered.size() != aircraft.size()) {
            status = std::format("Aircraft: {} / {}  Zoom: {:.1f}  Lat: {:.4f}  Lon: {:.4f}",
                                 filtered.size(), aircraft.size(), zoom, clat, clon);
        } else {
            status = std::format("Aircraft: {}  Zoom: {:.1f}  Lat: {:.4f}  Lon: {:.4f}",
                                 aircraft.size(), zoom, clat, clon);
        }
        list.fill_rect(0, static_cast<float>(h - 20), static_cast<float>(map_w), static_cast<float>(h),
                       adsb_draw::Color{0, 0, 0, 153});
        adsb_draw::text(list, 4, h - 5, status, adsb_draw::style(font, 0.4, {255, 255, 255}));
    }

    // render aircraft list panel
    if (s->cur.show_list && panel_w > 0) {
        adsb_list::render_aircraft_list(
            list, font, map_w, 0, panel_w, h,
            filtered, s->list_state, s->flag_cache,
            s->first_seen, now,
            s->cur.unit_system);
        // publish the post-sort, post-filter row order so the click hit-test
        // (interactions.cpp) maps row N to the same plane that was drawn there.
        s->panel_aircraft = filtered;
    } else {
        s->panel_aircraft.clear();
    }

    // render click info panel (on top of everything)
    uint32_t info_icao = s->info_panel_icao.load(std::memory_order_relaxed);
    if (info_icao != 0) {
        // find screen position of the selected aircraft from the just-rendered positions
        int build_idx = s->screen_pos_active.load(std::memory_order_relaxed);
        const auto& positions = s->screen_pos[build_idx];
        float ax = -1.0f, ay = -1.0f;
        bool found = false;
        for (const auto& sp : positions) {
            if (sp.icao == info_icao) {
                ax = static_cast<float>(sp.px_x);
                ay = static_cast<float>(sp.px_y);
                found = true;
                break;
            }
        }
        if (found) {
            adsb_tooltip::render_tooltip(
                list, font, w, h, info_icao, ax, ay,
                filtered, s->flag_cache, s->first_seen,
                s->photo_cache,
                s->cur.info_font_size,
                s->cur.info_panel_opacity,
                has_gps, gps_lat, gps_lon,
                s->cur.unit_system);
        } else {
            // aircraft no longer visible — dismiss the panel
            s->info_panel_icao.store(0, std::memory_order_relaxed);
        }
    }

    // airport click panel (mutually exclusive with the aircraft panel —
    // interactions.cpp clears one when the other is set). Gate on the
    // async load flag so we don't deref a half-initialized vector.
    int32_t info_airport_idx = s->info_panel_airport_idx.load(std::memory_order_relaxed);
    if (info_airport_idx >= 0
        && s->reference_loaded.load(std::memory_order_acquire)
        && static_cast<size_t>(info_airport_idx) < s->reference.airports.size()) {
        int ap_build_idx = s->airport_screen_pos_active.load(std::memory_order_relaxed);
        const auto& ap_positions = s->airport_screen_pos[ap_build_idx];
        float ax = -1.0f, ay = -1.0f;
        bool found = false;
        for (const auto& sp : ap_positions) {
            if (sp.airport_idx == info_airport_idx) {
                ax = static_cast<float>(sp.px_x);
                ay = static_cast<float>(sp.px_y);
                found = true;
                break;
            }
        }
        if (found) {
            adsb_airport_tooltip::render_airport_tooltip(
                list, font, w, h, info_airport_idx, ax, ay,
                s->reference.airports,
                s->flag_cache,
                s->cur.info_font_size,
                s->cur.info_panel_opacity,
                s->cur.unit_system);
        } else {
            // airport scrolled off — dismiss
            s->info_panel_airport_idx.store(-1, std::memory_order_relaxed);
        }
    }

    // Waypoint click panel — anchor at the marker pixel. Waypoints don't
    // have a cached screen-position buffer (the on-map pass projects them on
    // the fly under waypoints_mutex) so we project once here. Drop the panel
    // when the waypoint scrolls off-screen, matching the airspace behavior.
    {
        int32_t info_wp_idx = s->info_panel_waypoint_idx.load(std::memory_order_relaxed);
        if (info_wp_idx >= 0) {
            std::lock_guard lock(s->waypoints_mutex);
            if (static_cast<size_t>(info_wp_idx) < s->waypoints.size()) {
                const auto& wp = s->waypoints[info_wp_idx];
                auto anchor = geo_to_pixel(wp.lat, wp.lon, zoom, origin_tx, origin_ty);
                if (anchor.x < -50 || anchor.x > map_w + 50 ||
                    anchor.y < -50 || anchor.y > h + 50) {
                    s->info_panel_waypoint_idx.store(-1, std::memory_order_relaxed);
                } else {
                    adsb_waypoint_tooltip::render_waypoint_tooltip(
                        list, font, w, h, info_wp_idx,
                        static_cast<float>(anchor.x),
                        static_cast<float>(anchor.y),
                        s->waypoints,
                        s->cur.map_style,
                        s->cur.info_font_size,
                        s->cur.info_panel_opacity,
                        s->cur.unit_system);
                }
            } else {
                s->info_panel_waypoint_idx.store(-1, std::memory_order_relaxed);
            }
        }
    }

    // Airspace click panel — anchor at the polygon centroid (arithmetic mean
    // of ring vertices) projected to the canvas. When the centroid lands
    // off-screen we dismiss the panel so the tooltip doesn't render in a
    // random corner while the user pans away.
    int32_t info_airspace_idx = s->info_panel_airspace_idx.load(std::memory_order_relaxed);
    if (info_airspace_idx >= 0
        && s->reference_loaded.load(std::memory_order_acquire)
        && static_cast<size_t>(info_airspace_idx) < s->reference.airspaces.size()) {
        const auto& a = s->reference.airspaces[info_airspace_idx];
        if (!a.ring_lat_lon.empty()) {
            double sum_lat = 0, sum_lon = 0;
            for (const auto& [la, lo] : a.ring_lat_lon) { sum_lat += la; sum_lon += lo; }
            double cx_lat = sum_lat / a.ring_lat_lon.size();
            double cx_lon = sum_lon / a.ring_lat_lon.size();
            auto anchor_px = geo_to_pixel(cx_lat, cx_lon, zoom, origin_tx, origin_ty);
            if (anchor_px.x < -100 || anchor_px.x > map_w + 100 ||
                anchor_px.y < -100 || anchor_px.y > h + 100) {
                s->info_panel_airspace_idx.store(-1, std::memory_order_relaxed);
            } else {
                adsb_airspace_tooltip::render_airspace_tooltip(
                    list, font, w, h, info_airspace_idx,
                    static_cast<float>(anchor_px.x),
                    static_cast<float>(anchor_px.y),
                    s->reference.airspaces,
                    s->cur.info_font_size,
                    s->cur.info_panel_opacity,
                    s->cur.unit_system);
            }
        }
    }

    // Airspace hover label — a one-liner at the cursor showing the name +
    // type of the airspace under it. Hidden while the click panel is open
    // for that same airspace (avoid two labels for one polygon).
    {
        int32_t hov = s->hover_airspace_idx.load(std::memory_order_relaxed);
        int32_t pnl = s->info_panel_airspace_idx.load(std::memory_order_relaxed);
        if (hov >= 0 && hov != pnl
            && s->reference_loaded.load(std::memory_order_acquire)
            && static_cast<size_t>(hov) < s->reference.airspaces.size()) {
            const auto& a = s->reference.airspaces[hov];
            std::string label = a.name.empty() ? std::string{"<unnamed>"} : a.name;
            if (!a.type_label.empty()) label += "  [" + a.type_label + "]";

            float hx = s->hover_px_x.load(std::memory_order_relaxed);
            float hy = s->hover_px_y.load(std::memory_order_relaxed);

            auto outline = map_contrast_color(s->cur.map_style);
            auto halo    = map_halo_color(s->cur.map_style);
            const auto st = adsb_draw::style(font, 0.35, outline);
            const int tw = static_cast<int>(std::lround(adsb_draw::text_width(label, st)));
            const int th = static_cast<int>(std::lround(adsb_draw::text_height(st)));

            int lx = static_cast<int>(hx) + 14;
            int ly = static_cast<int>(hy) + 14;
            if (lx + tw > map_w - 4)  lx = static_cast<int>(hx) - tw - 6;
            if (ly + th > h - 4)      ly = static_cast<int>(hy) - 6;

            adsb_draw::text_outlined(list, lx, ly, label, st, halo);
        }
    }

    // Right-click delete confirmation modal — drawn last so it sits on top
    // of every other layer. Acquire-load pairs with the release-store in the
    // event handler so anchor / name writes are visible here.
    if (s->delete_confirm_idx.load(std::memory_order_acquire) >= 0) {
        adsb_waypoint_delete::render(list, font, w, h,
                                     s->delete_confirm_anchor_x,
                                     s->delete_confirm_anchor_y,
                                     s->delete_confirm_name);
    }

}

// The engine runs a node with no frame input in a tight loop: pace it.
static void end_frame(MapDisplayState* s)
{
    s->sequence++;
    // a drag moves the centre on every mouse move: the host hears where it ended
    if (!s->drag_active.load(std::memory_order_relaxed)) push_view(s);
    std::this_thread::sleep_for(std::chrono::milliseconds(33));
}

static int process(SpcPluginInstance* inst, SpcStep* step)
{
    auto* s = state(inst);
    if (step->output_count < 1) return -1;
    build_frame(s, step, false);

    const int w = s->cur.width;
    const int h = s->cur.height;
    const int map_w = s->bg_rgb.cols;
    SpcFrame* out = s->host.frame(0, static_cast<uint32_t>(w), static_cast<uint32_t>(h), SPC_PIXEL_FORMAT_RGB24);
    if (!out) {
        if (s->canvas.rows != h || s->canvas.cols != w) s->canvas = cv::Mat(h, w, CV_8UC3);
        spc::mat_to_frame(s->canvas, &s->output_frame, SPC_PIXEL_FORMAT_RGB24, 0, 0);
        out = &s->output_frame;
    }
    for (int y = 0; y < h; ++y) {
        uint8_t* row = out->data + static_cast<size_t>(y) * out->stride;
        std::memcpy(row, s->bg_rgb.ptr(y), static_cast<size_t>(map_w) * 3);
        std::memset(row + static_cast<size_t>(map_w) * 3, 200, static_cast<size_t>(w - map_w) * 3);
    }
    s->cpu_drawer.draw(out->data, out->width, out->height, out->stride, SPC_PIXEL_FORMAT_RGB24, s->list);

    out->sequence = s->sequence;
    out->timestamp_ns = spc::clock::now_utc_ns(s->host);
    step->outputs[0].type = SPC_DATA_FRAME;
    step->outputs[0].frame = out;
    end_frame(s);
    return 0;
}

#ifdef SPC_HAS_VULKAN
static int record_gpu(SpcPluginInstance* inst, SpcStep* step, SpcGpuPass* pass)
{
    auto* s = state(inst);
    if (!step || !pass || step->output_count < 1 || !s->gpu_ready) return -1;
    build_frame(s, step, true);

    const auto w = static_cast<uint32_t>(s->cur.width);
    const auto h = static_cast<uint32_t>(s->cur.height);
    const uint32_t stride = spc::draw::GpuDrawer::output_stride(w, SPC_PIXEL_FORMAT_RGB24);
    const uint64_t bytes = static_cast<uint64_t>(stride) * h;
    SpcFrame* out = s->host.frame(0, w, h, SPC_PIXEL_FORMAT_RGB24);
    if (!out) return -1;
    const SpcGpuEdgeBuffer slot = spc::acquire_ringed_output(pass, 0, w, h, stride, SPC_PIXEL_FORMAT_RGB24,
                                                             bytes, bytes);
    if (!slot.device_buffer || slot.gpu_handle == 0) {
        s->host.release_frame(out);
        return -1;
    }
    const spc::draw::GpuFrame dst{static_cast<VkBuffer>(slot.device_buffer), w, h, stride,
                                  SPC_PIXEL_FORMAT_RGB24};
    if (!s->gpu_drawer.record(*s->vk, pass, {}, dst, s->list, {}, spc::draw::Filter::Area, {200, 200, 200, 255})) {
        s->host.release_frame(out);
        return -1;
    }

    out->gpu_handle = slot.gpu_handle;
    out->gpu_flags |= SPC_GPU_FLAG_RESIDENT;
    out->sequence = s->sequence;
    out->timestamp_ns = spc::clock::now_utc_ns(s->host);
    step->outputs[0].type = SPC_DATA_FRAME;
    step->outputs[0].frame = out;
    end_frame(s);
    return 0;
}
#endif

#ifdef SPC_HAS_VULKAN
SPC_EXPORT_PLUGIN_SLOTS(
    .info              = info,
    .create_instance   = create_instance,
    .destroy_instance  = destroy_instance,
    .params_changed    = params_changed,
    .param_rules       = param_rules,
    .process           = process,
    .start             = start,
    .stop              = stop,
    .set_host_services = set_host_services,
    .ui_input          = ui_input,
    .record_gpu        = record_gpu,
    .interrupt         = request_stop)
#else
SPC_EXPORT_PLUGIN_SLOTS(
    .info              = info,
    .create_instance   = create_instance,
    .destroy_instance  = destroy_instance,
    .params_changed    = params_changed,
    .param_rules       = param_rules,
    .process           = process,
    .start             = start,
    .stop              = stop,
    .set_host_services = set_host_services,
    .ui_input          = ui_input,
    .interrupt         = request_stop)
#endif
