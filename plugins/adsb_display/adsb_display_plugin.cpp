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
#include <spc_ui_text.h>

SPC_PLUGIN_CAST(MapDisplayState)
SPC_PLUGIN_HOST_SERVICES(MapDisplayState, host)

// convenience alias for the GUI-settable parameter block (H6)
using Params = MapDisplayState::Params;

// ── descriptor ─────────────────────────────────────────────────────────

SPC_PLUGIN_DESCRIPTOR(
    spc::DescriptorBuilder("adsb_display", "ADS-B Display", "ADS-B/Visualization")
        .author("Speculor").version("0.1.0")
        .description("Renders ADS-B aircraft positions on an OpenStreetMap tile map")
        .maturity(SPC_MATURITY_STABLE)
        .tags({"ads-b", "tracking"})
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
        .input("enrichment_in", "Enrichment", SPC_DATA_RECORD, 1, SPC_CONSUME_NON_BLOCKING)
        .input_table("gps_in", "GPS Position", {
            {"latitude",  SPC_FIELD_FLOAT64},
            {"longitude", SPC_FIELD_FLOAT64},
        }, 1, SPC_CONSUME_NON_BLOCKING)
        .output("map_out", "Map", SPC_DATA_FRAME)
        .enum_param("view_mode", "View Mode", {"Auto", "Manual"}, 0, "View")
            .param_description("Auto centers on all aircraft, Manual uses fixed coordinates")
        .float64_param("center_lat", "Center Latitude", -90.0, 90.0, 51.5, 0.001, "View")
            .param_description("Map center latitude in Manual view mode")
        .float64_param("center_lon", "Center Longitude", -180.0, 180.0, -0.1, 0.001, "View")
            .param_description("Map center longitude in Manual view mode")
        .float_param("zoom", "Zoom", 2.0f, 18.0f, 6.0f, 0.1f, "View")
            .param_description("Map zoom level (higher = more detail, smaller area)")
        .float_param("radius", "Radius (km)", 1.0f, 5000.0f, 50.0f, 1.0f, "View")
            .param_description("Display radius in kilometers for range ring calculation")
        .int_param("width", "Width", 320, 3840, 1200, 10, "View")
            .param_description("Output frame width in pixels")
        .int_param("height", "Height", 240, 2160, 800, 10, "View")
            .param_description("Output frame height in pixels")
        .enum_param("map_style", "Map Style",
                    {"Standard", "Humanitarian", "Dark", "Light", "Voyager", "Topo", "Custom"},
                    2, "Tiles")
            .param_description("Map tile source and visual style")
        .string_param("custom_server", "Custom Server", "tile.openstreetmap.org", "Tiles")
            .param_description("Custom tile server hostname (e.g. tile.openstreetmap.org)")
        .float_param("icon_scale", "Icon Scale", 0.3f, 5.0f, 1.0f, 0.1f, "Display")
            .param_description("Aircraft icon size multiplier")
        .float_param("font_size", "Font Size", 0.1f, 2.0f, 0.35f, 0.05f, "Display")
            .param_description("Aircraft label font size multiplier")
        .enum_param("unit_system", "Units", {"Imperial", "Metric"}, 1, "Display")
            .param_description("Unit system: Imperial (ft, kts, fpm) or Metric (m, km/h, m/min)")
        .bool_param("show_labels", "Show Labels", true, "Display")
            .param_description("Show callsign and altitude labels on aircraft")
        .bool_param("show_trails", "Show Trails", true, "Display")
            .param_description("Draw position history trail behind each aircraft")
        .int_param("trail_length", "Trail Length", 5, 300, 30, 5, "Display")
            .param_description("Number of trail points to keep per aircraft")
        .bool_param("show_trend_vector", "Show Trend Vector", true, "Display")
            .param_description("Line ahead of each aircraft projecting its position forward")
        .float_param("trend_seconds", "Trend Seconds", 10.0f, 180.0f, 15.0f, 5.0f, "Display")
            .param_description("How many seconds ahead the trend-vector line projects")
        .bool_param("show_vs_indicator", "Show Vertical Speed", true, "Display")
            .param_description("Climb/descent arrow + rate next to the altitude label")
        .bool_param("show_staleness_fade", "Fade Stale Tracks", true, "Display")
            .param_description("Fade aircraft whose data is going stale before they expire")
        .int_param("filter_alt_min", "Min Altitude (ft)", 0, 60000, 0, 100, "Filters")
            .param_description("Hide airborne aircraft below this altitude (feet, independent of unit system)")
        .int_param("filter_alt_max", "Max Altitude (ft)", 0, 60000, 0, 100, "Filters")
            .param_description("Hide airborne aircraft above this altitude (feet, 0 = unlimited)")
        .float_param("filter_distance_max_km", "Max Distance (km)", 0.0f, 5000.0f, 0.0f, 1.0f, "Filters")
            .param_description("Hide aircraft farther than this from the receiver (0 = unlimited)")
        .bool_param("filter_show_ground", "Show Ground Traffic", true, "Filters")
            .param_description("Include aircraft reporting on-ground status")
        .bool_param("filter_military_only", "Military Only", false, "Filters")
            .param_description("Show only aircraft flagged as military in the ICAO DB")
        .string_param("filter_callsign_substr", "Callsign Contains", "", "Filters")
            .param_description("Show only aircraft whose callsign contains this text (case-insensitive; empty = all)")
        .bool_param("show_airports", "Show Airports", true, "Airports")
            .param_description("Plot airport markers from the fetched reference dataset")
        .enum_param("airport_min_type", "Airport Detail",
                    {"Large only", "Large + Medium", "All"}, 1, "Airports")
            .param_description("How many airport size tiers to draw (small airports and heliports also gate on zoom)")
        .bool_param("show_origin_dest_highlight", "Highlight Origin/Dest", true, "Airports")
            .param_description("For the selected aircraft, accent its origin and destination airports and draw a line to the destination")
        .bool_param("show_airspace", "Show Airspace", false, "Airspace")
            .param_description("Overlay airspace polygons (FIR/CTR/TMA/P-R-D/MATZ) if data was fetched at build time")
        .enum_param("airspace_classes", "Airspace Classes",
                    {"None", "Controlled", "Restricted", "All"}, 3, "Airspace")
            .param_description("Which airspace categories to show. Controlled = FIR/CTR/TMA etc; Restricted = P/R/D/MATZ.")
        .float_param("airspace_opacity", "Airspace Opacity", 0.0f, 1.0f, 0.10f, 0.05f, "Airspace")
            .param_description("Fill opacity for airspace polygons (outlines always full opacity)")
        .int_param("airspace_altitude_ft", "Altitude Filter (ft)", 0, 60000, 0, 500, "Airspace")
            .param_description("Show only airspaces whose floor/ceiling bracket this altitude. 0 = no filter.")
        .bool_param("show_range_rings", "Show Range Rings", true, "Range Rings")
            .param_description("Draw concentric distance rings on the map")
        .float_param("range_ring_interval", "Interval (km)", 1.0f, 100.0f, 20.0f, 1.0f, "Range Rings")
            .param_description("Distance between range rings in kilometers")
        .color_param("range_ring_color", "Ring Color", 0x00FF00FF, "Range Rings")
            .param_description("Range ring line color")
        .int_param("range_ring_thickness", "Ring Thickness", 1, 5, 1, 1, "Range Rings")
            .param_description("Range ring line thickness in pixels")
        .enum_param("range_ring_style", "Ring Style", {"Solid", "Dashed", "Dotted"}, 0, "Range Rings")
            .param_description("Range ring line style")
        .float_param("range_ring_opacity", "Ring Opacity", 0.0f, 1.0f, 0.15f, 0.05f, "Range Rings")
            .param_description("Range ring transparency (0 = invisible, 1 = opaque)")
        .float_param("range_ring_font_size", "Ring Font Size", 0.1f, 2.0f, 0.30f, 0.05f, "Range Rings")
        // Max Range group
        .bool_param("show_max_range", "Show Max Range", true, "Max Range")
            .param_description("Display a circle at the maximum distance any aircraft has been seen")
        .color_param("max_range_color", "Color", 0xFF6600FF, "Max Range")
        .float_param("max_range_opacity", "Opacity", 0.0f, 1.0f, 0.25f, 0.05f, "Max Range")
        .float_param("max_range_font_size", "Font Size", 0.1f, 2.0f, 0.30f, 0.05f, "Max Range")
            .param_description("Font size for range ring distance labels")
        .float_param("info_font_size", "Font Size", 0.2f, 0.6f, 0.30f, 0.02f, "Info Panel")
            .param_description("Font size for the aircraft info panel")
        .float_param("info_panel_opacity", "Panel Opacity", 0.0f, 1.0f, 0.92f, 0.05f, "Info Panel")
            .param_description("Opacity of the aircraft info panel background (0 = transparent, 1 = opaque)")
        .bool_param("show_list", "Show Aircraft List", true, "Aircraft List")
            .param_description("Show scrollable aircraft list panel on the right side")
        .int_param("list_width", "List Width", 150, 500, 220, 10, "Aircraft List")
            .param_description("Width of the aircraft list panel in pixels")
        .list_param("waypoints", "Waypoints", "Map Waypoints")
            .param_description("Points of interest drawn on the map (right-click map to add)")
            .list_column("name", "Name", SPC_PARAM_STRING)
            .list_column_enum("type", "Type", {"Waypoint", "Airport", "Military Base", "VOR", "NDB"})
            .list_column("lat", "Latitude", SPC_PARAM_FLOAT64, -90.0, 90.0, 0.0001)
            .list_column("lon", "Longitude", SPC_PARAM_FLOAT64, -180.0, 180.0, 0.0001)
            .list_column("alt", "Altitude (ft)", SPC_PARAM_FLOAT64, 0.0, 100000.0, 1.0)
        .streaming().frame_alloc().interactive()
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
        return 0;
    }
    if (std::strcmp(name, "radius") == 0 && value->type == SPC_PARAM_FLOAT) {
        s->radius.store(value->float_val.value, std::memory_order_relaxed);
        s->interactive_view.store(false, std::memory_order_relaxed);
        const Params p = s->params.snapshot();
        sync_zoom_from_radius(s, p.width, p.height);
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

static int get_parameter(SpcPluginInstance* inst, const char* name,
                         SpcParameterDesc* out)
{
    auto* s = state(inst);
    const Params p = s->params.snapshot();
    bool auto_view = (s->view_mode.load(std::memory_order_relaxed) == 0);

    if (spc::try_get_enum(name, out, "view_mode", s->view_mode.load(std::memory_order_relaxed))) return 0;
    if (spc::try_get_float64(name, out, "center_lat", s->center_lat.load(std::memory_order_relaxed))) {
        if (auto_view) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_float64(name, out, "center_lon", s->center_lon.load(std::memory_order_relaxed))) {
        if (auto_view) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_float(name, out, "zoom", s->zoom.load(std::memory_order_relaxed))) {
        if (auto_view) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_float(name, out, "radius", s->radius.load(std::memory_order_relaxed))) {
        if (auto_view) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_int(name, out, "width", p.width)) return 0;
    if (spc::try_get_int(name, out, "height", p.height)) return 0;
    if (spc::try_get_enum(name, out, "map_style", p.map_style)) return 0;
    if (spc::try_get_string(name, out, "custom_server", p.custom_server)) {
        if (p.map_style != 6) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_float(name, out, "icon_scale", p.icon_scale)) return 0;
    if (spc::try_get_float(name, out, "font_size", p.font_size)) return 0;
    if (spc::try_get_enum(name, out, "unit_system", p.unit_system)) return 0;
    if (spc::try_get_bool(name, out, "show_labels", p.show_labels)) return 0;
    if (spc::try_get_bool(name, out, "show_trails", p.show_trails)) return 0;
    if (spc::try_get_int(name, out, "trail_length", p.trail_length)) {
        if (!p.show_trails) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_bool(name, out, "show_trend_vector", p.show_trend_vector)) return 0;
    if (spc::try_get_float(name, out, "trend_seconds", p.trend_seconds)) {
        if (!p.show_trend_vector) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_bool(name, out, "show_vs_indicator", p.show_vs_indicator)) return 0;
    if (spc::try_get_bool(name, out, "show_staleness_fade", p.show_staleness_fade)) return 0;
    if (spc::try_get_int(name, out, "filter_alt_min", p.filter_alt_min)) return 0;
    if (spc::try_get_int(name, out, "filter_alt_max", p.filter_alt_max)) return 0;
    if (spc::try_get_float(name, out, "filter_distance_max_km", p.filter_distance_max_km)) return 0;
    if (spc::try_get_bool(name, out, "filter_show_ground", p.filter_show_ground)) return 0;
    if (spc::try_get_bool(name, out, "filter_military_only", p.filter_military_only)) return 0;
    if (spc::try_get_string(name, out, "filter_callsign_substr", p.filter_callsign_substr)) return 0;
    if (spc::try_get_bool(name, out, "show_airspace", p.show_airspace)) return 0;
    if (spc::try_get_enum(name, out, "airspace_classes", p.airspace_classes)) {
        if (!p.show_airspace) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_float(name, out, "airspace_opacity", p.airspace_opacity)) {
        if (!p.show_airspace) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_int(name, out, "airspace_altitude_ft", p.airspace_altitude_ft)) {
        if (!p.show_airspace) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_bool(name, out, "show_airports", p.show_airports)) return 0;
    if (spc::try_get_enum(name, out, "airport_min_type", p.airport_min_type)) {
        if (!p.show_airports) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_bool(name, out, "show_origin_dest_highlight", p.show_origin_dest_highlight)) {
        if (!p.show_airports) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_bool(name, out, "show_range_rings", p.show_range_rings)) return 0;
    if (spc::try_get_float(name, out, "range_ring_interval", p.range_ring_interval)) {
        if (!p.show_range_rings) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_color(name, out, "range_ring_color", p.range_ring_color)) {
        if (!p.show_range_rings) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_int(name, out, "range_ring_thickness", p.range_ring_thickness)) {
        if (!p.show_range_rings) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_enum(name, out, "range_ring_style", p.range_ring_style)) {
        if (!p.show_range_rings) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_float(name, out, "range_ring_opacity", p.range_ring_opacity)) {
        if (!p.show_range_rings) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_float(name, out, "range_ring_font_size", p.range_ring_font_size)) {
        if (!p.show_range_rings) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_bool(name, out, "show_max_range", p.show_max_range)) return 0;
    if (spc::try_get_color(name, out, "max_range_color", p.max_range_color)) {
        if (!p.show_max_range) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_float(name, out, "max_range_opacity", p.max_range_opacity)) {
        if (!p.show_max_range) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_float(name, out, "max_range_font_size", p.max_range_font_size)) {
        if (!p.show_max_range) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_float(name, out, "info_font_size", p.info_font_size)) return 0;
    if (spc::try_get_float(name, out, "info_panel_opacity", p.info_panel_opacity)) return 0;
    if (spc::try_get_bool(name, out, "show_list", p.show_list)) return 0;
    if (spc::try_get_int(name, out, "list_width", p.list_width)) {
        if (!p.show_list) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (std::strcmp(name, "waypoints") == 0) {
        out->type = SPC_PARAM_LIST;
        std::lock_guard lock(s->waypoints_mutex);
        out->list_val.row_count = static_cast<int32_t>(s->waypoints.size());
        return 0;
    }
    return -1;
}

// ── list parameter handlers ────────────────────────────────────────────

static int get_list_rows_impl(MapDisplayState* s, const char* name,
                                SpcListRow* out, int32_t max, int32_t* count)
{
    if (std::strcmp(name, "waypoints") != 0) return -1;
    std::lock_guard lock(s->waypoints_mutex);
    int32_t n = std::min(static_cast<int32_t>(s->waypoints.size()), max);
    for (int32_t i = 0; i < n; ++i) {
        std::memset(&out[i], 0, sizeof(SpcListRow));
        std::strncpy(out[i].cells[0].string_val, s->waypoints[i].name, SPC_LIST_CELL_STRING_MAX - 1);
        out[i].cells[1].enum_val = s->waypoints[i].type;
        out[i].cells[2].float64_val = s->waypoints[i].lat;
        out[i].cells[3].float64_val = s->waypoints[i].lon;
        out[i].cells[4].float64_val = s->waypoints[i].alt;
    }
    *count = n;
    return 0;
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

SPC_PLUGIN_LIST_HANDLERS(MapDisplayState, get_list_rows_impl, set_list_rows_impl)

// ── streaming ──────────────────────────────────────────────────────────

static int start(SpcPluginInstance* inst)
{
    auto* s = state(inst);
    const Params p = s->params.snapshot();
    s->frame_number = 0;
    s->offsets_resolved = false;
    s->gps_offsets_resolved = false;
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
    s->applied_map_style = p.map_style;
    std::strncpy(s->applied_custom_server, p.custom_server, sizeof(s->applied_custom_server) - 1);
    s->applied_custom_server[sizeof(s->applied_custom_server) - 1] = '\0';
    s->download_queue.start(server, &s->host.cached_log);
    s->photo_queue.start(&s->host.cached_log);
    s->photo_cache.clear();
    s->enrichment_cache.clear();
    s->max_range_km = 0.0;
    // clear any pending param reconfiguration — start applied current params
    s->params_dirty.store(false, std::memory_order_release);

    // One-shot load of reference data (airports + airspace). CMake copies
    // Reference data (airports + airspace) is loaded asynchronously on a
    // background thread — even from the binary cache it's tens of MB, and a
    // fresh first-run JSON parse is 10-30 s. Doing this on start() would
    // freeze the engine. The airports/airspace layers and related tooltip
    // code all gate on `reference_loaded.load(acquire)` before touching the
    // data, so rendering is safe while the thread is still working.
    if (!s->reference_loaded.load(std::memory_order_acquire)
        && !s->loader_thread.joinable()) {
        auto data_dir = spc::plugin_data::own_module_dir() / "data" / "openaip";
        auto airports_path = (data_dir / "airports.json").string();
        auto airspace_path = (data_dir / "airspace.json").string();

        s->loader_thread = std::jthread([s, airports_path, airspace_path] {
            auto t0 = std::chrono::steady_clock::now();
            s->reference = load_reference_data(airports_path, airspace_path);
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

static int stop(SpcPluginInstance* inst)
{
    auto* s = state(inst);
    s->download_queue.stop();
    s->photo_queue.stop();
    s->flag_cache.clear();
    s->first_seen.clear();
    s->screen_pos[0].clear();
    s->screen_pos[1].clear();
    SPC_LOG_INFO(&s->host.cached_log, "adsb display stopped (%llu frames)",
                 static_cast<unsigned long long>(s->frame_number));
    return 0;
}

SPC_PLUGIN_EVENT_HANDLER(MapDisplayState, handle_event)

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

static int process(SpcPluginInstance* inst, const SpcData* inputs,
                   uint32_t input_count, SpcData* outputs, uint32_t output_count)
{
    auto* s = state(inst);
    if (output_count < 1) return -1;

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
    if (read_gps_position(s, inputs, input_count, gps_lat, gps_lon)) {
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

    // create canvas (full width including panel area). Reuse the existing
    // Mat across frames — only allocate when missing or when the size /
    // type changed (parameter resize). Saves a ~2.8 MB allocation +
    // initialization every process() call.
    if (s->canvas.empty() || s->canvas.rows != h || s->canvas.cols != w ||
        s->canvas.type() != CV_8UC3) {
        s->canvas = cv::Mat(h, w, CV_8UC3, cv::Scalar(200, 200, 200));
    } else {
        s->canvas.setTo(cv::Scalar(200, 200, 200));
    }

    // ── sticky background cache ────────────────────────────────────────
    // The static layers (tiles + airspace + airports + range rings + max
    // range) depend only on view + style + selection state — none of which
    // change frame-to-frame on a stable view. Cache the rendered map_rect
    // and reuse it whenever the BackgroundKey is unchanged. On a steady
    // view this turns ~22 ms of per-frame OpenCV work into a single
    // ~2.8 MB memcpy.
    cv::Rect map_rect(0, 0, map_w, h);
    BackgroundKey cur_key = compute_bg_key(s, s->cur, zoom, origin_tx, origin_ty,
                                           map_w, h, gps_lat, gps_lon, has_gps);

    bool cache_hit = s->bg_key_valid &&
                     cur_key == s->bg_key &&
                     !s->bg_cache.empty() &&
                     s->bg_cache.rows == h &&
                     s->bg_cache.cols == map_w &&
                     s->bg_cache.type() == s->canvas.type();

    if (cache_hit) {
        s->bg_cache.copyTo(s->canvas(map_rect));
    } else {
        // render map into the left portion (0..map_w)
        composite_tiles(s, s->canvas, zoom, origin_tx, origin_ty, map_w, h);
        // airspace sits between tiles and aircraft so polygons color the map
        // without drowning out icons drawn later
        render_airspace_layer(s, zoom, origin_tx, origin_ty, map_w, h);
        // airport markers render above airspace but below aircraft so they
        // read as landmarks rather than as traffic
        render_airports_layer(s, zoom, origin_tx, origin_ty, map_w, h);
        if (has_gps) {
            render_range_rings(s, gps_lat, gps_lon, zoom, origin_tx, origin_ty, map_w, h);
            render_max_range(s, gps_lat, gps_lon, zoom, origin_tx, origin_ty, map_w, h);
        }

        // capture the rendered map area for the next frame's blit.
        s->canvas(map_rect).copyTo(s->bg_cache);
        s->bg_key = cur_key;
        s->bg_key_valid = true;
    }

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
            int marker_type = cv::MARKER_DIAMOND;
            switch (wp.type) {
                case 0: color = map_contrast_color(s->cur.map_style); marker_type = cv::MARKER_DIAMOND; break;  // waypoint: adaptive diamond
                case 1: color = {50, 180, 255};  marker_type = cv::MARKER_SQUARE;  break;  // airport: blue square
                case 2: color = {255, 60, 60};   marker_type = cv::MARKER_STAR;    break;  // military: red star
                case 3: color = {50, 200, 50};   marker_type = cv::MARKER_TILTED_CROSS; break; // VOR: green cross
                case 4: color = {255, 180, 50};  marker_type = cv::MARKER_TRIANGLE_UP;  break; // NDB: orange triangle
                default: color = {200, 200, 200}; break;
            }

            cv::drawMarker(s->canvas, {px, py}, color, marker_type, 12, 2, cv::LINE_AA);

            // draw name label with background
            if (wp.name[0] != '\0') {
                int baseline = 0;
                auto text_size = cv::getTextSize(wp.name, cv::FONT_HERSHEY_SIMPLEX, 0.35, 1, &baseline);
                int lx = px + 8, ly = py - 4;
                auto wp_label_bg = is_dark_map(s->cur.map_style)
                    ? cv::Scalar(0, 0, 0) : cv::Scalar(240, 240, 240);
                cv::rectangle(s->canvas, {lx - 1, ly - text_size.height - 1},
                             {lx + text_size.width + 1, ly + 2}, wp_label_bg, cv::FILLED);
                cv::putText(s->canvas, wp.name, {lx, ly},
                           cv::FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv::LINE_AA);
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

    render_aircraft_layer(s, filtered, zoom, origin_tx, origin_ty, map_w, h);

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
                cv::Point p0(static_cast<int>(ac_px.x),   static_cast<int>(ac_px.y));
                cv::Point p1(static_cast<int>(dest_px.x), static_cast<int>(dest_px.y));
                double dx = p1.x - p0.x, dy = p1.y - p0.y;
                double len = std::sqrt(dx * dx + dy * dy);
                if (len >= 5.0) {
                    constexpr double dash = 8.0, gap = 5.0;
                    double ux = dx / len, uy = dy / len;
                    cv::Scalar leader_color(255, 200, 40);
                    for (double d = 0; d < len; d += dash + gap) {
                        double d1 = std::min(d + dash, len);
                        cv::line(s->canvas,
                                 {static_cast<int>(p0.x + ux * d),
                                  static_cast<int>(p0.y + uy * d)},
                                 {static_cast<int>(p0.x + ux * d1),
                                  static_cast<int>(p0.y + uy * d1)},
                                 leader_color, 1, cv::LINE_AA);
                    }
                }
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
                cv::Point p0(static_cast<int>(gps_px.x), static_cast<int>(gps_px.y));
                cv::Point p1(static_cast<int>(ac_px.x), static_cast<int>(ac_px.y));
                // draw dashed line manually
                double dx = p1.x - p0.x, dy = p1.y - p0.y;
                double len = std::sqrt(dx * dx + dy * dy);
                if (len > 5.0) {
                    constexpr double dash = 8.0, gap = 5.0;
                    double ux = dx / len, uy = dy / len;
                    for (double d = 0; d < len; d += dash + gap) {
                        double d1 = std::min(d + dash, len);
                        cv::line(s->canvas,
                                 {static_cast<int>(p0.x + ux * d), static_cast<int>(p0.y + uy * d)},
                                 {static_cast<int>(p0.x + ux * d1), static_cast<int>(p0.y + uy * d1)},
                                 map_contrast_color(s->cur.map_style), 1, cv::LINE_AA);
                    }

                    // distance and bearing label at midpoint
                    double dist_km = haversine_km(gps_lat, gps_lon, ac.lat, ac.lon);
                    double brg = bearing_deg(gps_lat, gps_lon, ac.lat, ac.lon);
                    std::string dist_label;
                    if (dist_km < 1.0)
                        dist_label = std::format("{:.0f}m {:.0f}deg", dist_km * 1000.0, brg);
                    else
                        dist_label = std::format("{:.1f}km {:.0f}deg", dist_km, brg);

                    int mx = (p0.x + p1.x) / 2;
                    int my = (p0.y + p1.y) / 2;
                    int baseline = 0;
                    auto sz = cv::getTextSize(dist_label, cv::FONT_HERSHEY_SIMPLEX, 0.35, 1, &baseline);
                    auto dist_bg = is_dark_map(s->cur.map_style)
                        ? cv::Scalar(0x2e, 0x1e, 0x1e)
                        : cv::Scalar(0xf0, 0xf0, 0xf0);
                    cv::rectangle(s->canvas,
                                  {mx - sz.width / 2 - 2, my - sz.height - 2},
                                  {mx + sz.width / 2 + 2, my + 3},
                                  dist_bg, cv::FILLED);
                    cv::putText(s->canvas, dist_label,
                                {mx - sz.width / 2, my},
                                cv::FONT_HERSHEY_SIMPLEX, 0.35,
                                map_contrast_color(s->cur.map_style), 1, cv::LINE_AA);
                }
                break;
            }
        }
    }

    // map overlay buttons (top-left)
    {
        static const spc::ui::Theme th;
        s->ui_buttons.begin_frame(s->canvas, w, h);
        bool gps_clicked = s->ui_buttons.button(
            "Center GPS", false, has_gps ? th.green : th.surface2);
        bool fit_clicked = s->ui_buttons.button(
            "Fit All", false, !filtered.empty() ? th.blue : th.surface2);
        s->ui_buttons.end_frame();

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
    render_compass_rose(s, map_w, h);

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
        cv::Mat bar_roi = s->canvas(cv::Rect(0, h - 20, map_w, 20));
        bar_roi *= 0.4;
        cv::putText(s->canvas, status, {4, h - 5},
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, {255, 255, 255}, 1, cv::LINE_AA);
    }

    // render aircraft list panel
    if (s->cur.show_list && panel_w > 0) {
        adsb_list::render_aircraft_list(
            s->canvas, map_w, 0, panel_w, h,
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
                s->canvas, info_icao, ax, ay,
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
                s->canvas, info_airport_idx, ax, ay,
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
                        s->canvas, info_wp_idx,
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
                    s->canvas, info_airspace_idx,
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
            double fs = 0.35;
            int baseline = 0;
            auto sz = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, fs, 1, &baseline);

            int lx = static_cast<int>(hx) + 14;
            int ly = static_cast<int>(hy) + 14;
            if (lx + sz.width > map_w - 4)  lx = static_cast<int>(hx) - sz.width - 6;
            if (ly + sz.height > h - 4)     ly = static_cast<int>(hy) - 6;

            spc::ui::draw_text_outlined(s->canvas, label, {lx, ly}, fs, outline, halo);
        }
    }

    // Right-click delete confirmation modal — drawn last so it sits on top
    // of every other layer. Acquire-load pairs with the release-store in the
    // event handler so anchor / name writes are visible here.
    if (s->delete_confirm_idx.load(std::memory_order_acquire) >= 0) {
        adsb_waypoint_delete::render(s->canvas,
                                     s->delete_confirm_anchor_x,
                                     s->delete_confirm_anchor_y,
                                     s->delete_confirm_name);
    }

    // output frame — the canvas uses RGB-in-Scalar convention (Theme is RGB)
    // but imdecoded images (flags, tiles) are BGR, so swap those channels for output
    // approach: treat canvas as RGB throughout, convert decoded images on load instead
    auto now_ns = spc::clock::now_utc_ns(s->host);

    SpcFrame* pool_frame = s->host.acquire_frame(
        0, static_cast<uint32_t>(w), static_cast<uint32_t>(h), SPC_PIXEL_FORMAT_RGB24);

    if (pool_frame) {
        spc::copy_frame_rows(s->canvas.data, static_cast<uint32_t>(s->canvas.step[0]),
                             pool_frame->data, pool_frame->stride,
                             static_cast<uint32_t>(h));
        pool_frame->frame_number = s->frame_number;
        pool_frame->timestamp_ns = now_ns;
        outputs[0].type = SPC_DATA_FRAME;
        outputs[0].frame = pool_frame;
    } else {
        spc::mat_to_frame(s->canvas, &s->output_frame,
                          SPC_PIXEL_FORMAT_RGB24, s->frame_number, now_ns);
        outputs[0].type = SPC_DATA_FRAME;
        outputs[0].frame = &s->output_frame;
    }

    s->frame_number++;

    std::this_thread::sleep_for(std::chrono::milliseconds(33));
    return 0;
}

SPC_PLUGIN_VTABLE(
    .get_descriptor    = get_descriptor,
    .create_instance   = create_instance,
    .destroy_instance  = destroy_instance,
    .set_parameter     = set_parameter,
    .get_parameter     = get_parameter,
    .process           = process,
    .start             = start,
    .stop              = stop,
    .set_host_services = set_host_services,
    .on_input_event          = on_input_event,
    .get_list_rows     = get_list_rows,
    .set_list_rows     = set_list_rows
)
