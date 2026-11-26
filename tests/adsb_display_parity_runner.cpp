// adsb_display draws the same map on the GPU as on the CPU: the same aircraft
// and receiver position go to process() and to record_gpu() of the shipped
// plugin, in two nodes stepped alike, and the frames must agree: aircraft
// icons, trails, labels, range rings, the compass, the buttons, the list panel
// with its flags, and a clicked aircraft's info panel.
//
// The tile server is unreachable (placeholder tiles) and the airport and
// airspace layers are off: the reference data loads on a thread, so either
// node could draw it a frame earlier than the other.
//
// usage: spc_adsb_display_parity_tests <path-to-adsb_display>
// SKIPs (exit 0) without a Vulkan device.

#include "display_gpu_harness.h"

#include <testing/fake_host.h>

#include <gpu/gpu_host.h>

#include <speculor/schema_storage.h>
#include <speculor/table_helpers.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

using display_test::Image;
using spc::gpu::VulkanContext;

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    std::printf("  %s: %s\n", ok ? "ok  " : "FAIL", what.c_str());
    if (!ok) ++failures;
}

constexpr double kRxLat = 51.47, kRxLon = -0.45;
constexpr uint32_t kW = 900, kH = 560;

struct Aircraft {
    uint32_t icao;
    const char* callsign;
    const char* type_code;
    double north_km, east_km;
    int32_t alt_ft, baro_rate;
    float track, gs;
    uint8_t msg_source, category;
};

const Aircraft kAircraft[] = {
    {0x4ca7b1, "RYR12AB", "B738", 12.0, 18.0, 5600, -1200, 250.f, 280.f, 0, 3},
    {0x3c6754, "DLH4KM", "A320", -20.0, 30.0, 36000, 0, 95.f, 450.f, 0, 3},
    {0x40621d, "BAW2X", "B77W", 25.0, -15.0, 12000, 1800, 320.f, 330.f, 5, 5},
    {0x39c4a5, "AFR77", "A388", -8.0, -35.0, 22000, 0, 10.f, 470.f, 0, 5},
    {0x4b1812, "SWR9", "C172", 4.0, 6.0, 2500, 400, 180.f, 95.f, 0, 1},
};

struct AircraftTable {
    spc::SchemaStorage schema;
    SpcTable t{};

    AircraftTable() {
        const std::pair<const char*, SpcFieldType> fields[] = {
            {"icao_addr", SPC_FIELD_UINT32},  {"callsign", SPC_FIELD_STRING32}, {"alt_baro", SPC_FIELD_INT32},
            {"ground_speed", SPC_FIELD_FLOAT}, {"track", SPC_FIELD_FLOAT},       {"baro_rate", SPC_FIELD_INT32},
            {"latitude", SPC_FIELD_FLOAT64},  {"longitude", SPC_FIELD_FLOAT64}, {"category", SPC_FIELD_UINT8},
            {"msg_source", SPC_FIELD_UINT8},  {"rssi", SPC_FIELD_FLOAT},        {"registration", SPC_FIELD_STRING32},
            {"type_code", SPC_FIELD_STRING32}, {"is_on_ground", SPC_FIELD_BOOL},
        };
        for (const auto& [name, type] : fields) schema.add_field(name, type);
        uint32_t off[SPC_SCHEMA_MAX_FIELDS]{};
        uint32_t stride = 0;
        spc_schema_compute_offsets(schema.get(), off, &stride);
        spc_table_init(&t, stride, schema.get());
        const uint32_t n = sizeof(kAircraft) / sizeof(kAircraft[0]);
        (void)spc_table_resize(&t, n);
        const double km_per_deg_lon = 111.32 * std::cos(kRxLat * 3.141592653589793 / 180.0);
        for (uint32_t i = 0; i < n; ++i) {
            const Aircraft& a = kAircraft[i];
            spc_table_set_uint32(&t, i, off[0], a.icao);
            spc_table_set_string(&t, i, off[1], a.callsign, 32);
            spc_table_set_int32(&t, i, off[2], a.alt_ft);
            spc_table_set_float(&t, i, off[3], a.gs);
            spc_table_set_float(&t, i, off[4], a.track);
            spc_table_set_int32(&t, i, off[5], a.baro_rate);
            spc_table_set_float64(&t, i, off[6], kRxLat + a.north_km / 111.32);
            spc_table_set_float64(&t, i, off[7], kRxLon + a.east_km / km_per_deg_lon);
            spc_table_set_uint8(&t, i, off[8], a.category);
            spc_table_set_uint8(&t, i, off[9], a.msg_source);
            spc_table_set_float(&t, i, off[10], -15.f);
            spc_table_set_string(&t, i, off[11], "G-EUPT", 32);
            spc_table_set_string(&t, i, off[12], a.type_code, 32);
            spc_table_set_bool(&t, i, off[13], 0);
        }
    }
    ~AircraftTable() { spc_table_free(&t); }
};

struct GpsTable {
    spc::SchemaStorage schema;
    SpcTable t{};

    GpsTable() {
        schema.add_field("latitude", SPC_FIELD_FLOAT64);
        schema.add_field("longitude", SPC_FIELD_FLOAT64);
        uint32_t off[SPC_SCHEMA_MAX_FIELDS]{};
        uint32_t stride = 0;
        spc_schema_compute_offsets(schema.get(), off, &stride);
        spc_table_init(&t, stride, schema.get());
        (void)spc_table_resize(&t, 1);
        spc_table_set_float64(&t, 0, off[0], kRxLat);
        spc_table_set_float64(&t, 0, off[1], kRxLon);
    }
    ~GpsTable() { spc_table_free(&t); }
};

struct Value {
    const char* name;
    double number;
    const char* text = nullptr;
};

struct Setting {
    const char* name;
    std::vector<Value> values;
    bool click_first_row = false;
};

class Node {
public:
    bool open(const std::string& path, const Setting& s, AircraftTable& ac, GpsTable& gps) {
        if (!p_.load(path) || !p_.create()) {
            std::printf("  cannot load %s: %s\n", path.c_str(), p_.error().c_str());
            return false;
        }
        if (auto vk = VulkanContext::get_shared())
            host_.lend_gpu(spc::gpu::host_device(*vk),
                           {&spc::gpu::host_buffer, &spc::gpu::host_share_buffer,
                            &spc::gpu::host_unshare_buffer, &spc::gpu::host_read_buffer});
        host_.serve_tables(p_.descriptor());
        p_.set_host_services(host_.services());
        host_.as_input(0, ac.t);
        host_.as_input(2, gps.t);
        const std::vector<Value> common = {
            {"width", kW}, {"height", kH}, {"map_style", 5}, {"custom_server", 0, "127.0.0.1:9"},
            {"show_airports", 0}, {"show_airspace", 0},
        };
        for (const auto* vals : {&common, &s.values})
            for (const auto& v : *vals)
                if (!set(v)) {
                    std::printf("  cannot set %s\n", v.name);
                    return false;
                }
        return p_.start() == 0;
    }
    ~Node() { p_.stop(); }

    // a left click at the normalized (x, y), between steps
    void click(float x, float y) {
        if (!p_.vtable()->ui_input) return;
        SpcUiInput ev{};
        ev.type = SPC_UI_MOUSE_PRESS;
        ev.mouse.x = x;
        ev.mouse.y = y;
        ev.mouse.button = SPC_MOUSE_BUTTON_LEFT;
        p_.vtable()->ui_input(p_.instance(), &ev);
        ev.type = SPC_UI_MOUSE_RELEASE;
        p_.vtable()->ui_input(p_.instance(), &ev);
    }

    bool run_cpu(AircraftTable& ac, GpsTable& gps, Image& out) {
        SpcData inputs[3]{};
        fill(inputs, ac, gps);
        SpcData outputs[1]{};
        return p_.process(inputs, 3, outputs, 1) == 0 && display_test::read_frame(outputs[0].frame, out);
    }

    bool run_gpu(VulkanContext& ctx, AircraftTable& ac, GpsTable& gps, Image& out) {
        SpcData inputs[3]{};
        fill(inputs, ac, gps);
        return display_test::record_gpu(ctx, p_, inputs, 3, out);
    }

private:
    bool set(const Value& v) {
        SpcParameterDesc d{};
        if (p_.get_parameter(v.name, &d) != 0) return false;
        switch (d.type) {
        case SPC_PARAM_BOOL: d.bool_val.value = static_cast<int32_t>(v.number); break;
        case SPC_PARAM_INT: d.int_val.value = static_cast<int32_t>(v.number); break;
        case SPC_PARAM_ENUM: d.enum_val.value = static_cast<int32_t>(v.number); break;
        case SPC_PARAM_FLOAT: d.float_val.value = static_cast<float>(v.number); break;
        case SPC_PARAM_STRING:
            std::snprintf(d.string_val.value, sizeof(d.string_val.value), "%s", v.text ? v.text : "");
            break;
        default: return false;
        }
        return p_.set_parameter(v.name, &d) == 0;
    }

    static void fill(SpcData* inputs, AircraftTable& ac, GpsTable& gps) {
        inputs[0].type = SPC_DATA_TABLE;
        inputs[0].table = &ac.t;
        inputs[2].type = SPC_DATA_TABLE;
        inputs[2].table = &gps.t;
    }

    spc::testing::FakeHost host_;
    spc::testing::PluginUnderTest p_;
};

void compare(const std::string& plugin, VulkanContext& ctx, const Setting& s) {
    std::printf("%s\n", s.name);
    AircraftTable ac;
    GpsTable gps;
    Node cpu, gpu;
    const bool opened = cpu.open(plugin, s, ac, gps) && gpu.open(plugin, s, ac, gps);
    check(opened, "both nodes start");
    if (!opened) return;
    for (int frame = 0; frame < 3; ++frame) {
        Image a, b;
        const bool ran = cpu.run_cpu(ac, gps, a) && gpu.run_gpu(ctx, ac, gps, b);
        check(ran, "frame " + std::to_string(frame) + ": both paths draw it");
        if (!ran) return;
        if (frame == 0 && s.click_first_row) {
            // the list panel's first row, below its 22-pixel header
            const float x = (static_cast<float>(kW) - 100.f) / static_cast<float>(kW);
            const float y = 36.f / static_cast<float>(kH);
            cpu.click(x, y);
            gpu.click(x, y);
        }
        if (frame < 2) continue;
        check(a.w == b.w && a.h == b.h, "the same size, " + std::to_string(b.w) + "x" + std::to_string(b.h));
        const double wide = display_test::share_differing(a, b, 96);
        const double narrow = display_test::share_differing(a, b, 24);
        check(wide < 0.004, "pixels far apart: " + std::to_string(wide * 100) + "%");
        check(narrow < 0.02, "pixels apart by more than 24 levels: " + std::to_string(narrow * 100) + "%");
        size_t drawn = 0;
        for (size_t i = 0; i < b.px.size(); i += 3) drawn += b.px[i] != 200 || b.px[i + 1] != 200 || b.px[i + 2] != 200;
        check(drawn > 20000, "the GPU drew the map (" + std::to_string(drawn) + " pixels off the background)");
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <path-to-adsb_display>\n", argv[0]);
        return 2;
    }
    auto ctx = VulkanContext::get_shared();
    if (!ctx || !ctx->valid) {
        std::printf("SKIPPED: no Vulkan device\n");
        return 0;
    }
    const std::string plugin = argv[1];
    const Setting settings[] = {
        {"defaults, with the receiver's range rings", {}},
        {"a clicked aircraft's info panel, Noto Sans, imperial", {{"font", 1}, {"unit_system", 0}}, true},
        {"Roboto, dashed thick rings, large icons, no list",
         {{"font", 2}, {"range_ring_style", 1}, {"range_ring_thickness", 2}, {"icon_scale", 2.0},
          {"show_list", 0}}},
    };
    for (const auto& s : settings) compare(plugin, *ctx, s);

    std::printf("%s (%d failure%s)\n", failures ? "FAILED" : "PASSED", failures,
                failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
