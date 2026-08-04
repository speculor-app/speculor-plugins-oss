// adsb_display without a display.
//
// The rendering cannot be asserted headlessly, but the part that loses user
// data can: waypoints are a list parameter, and a list parameter is what a
// .speculor file stores and restores. If set_list_rows and get_list_rows
// disagree — a dropped column, a truncated string, a row silently discarded on
// overflow — a user's waypoints come back wrong after a reload, and nothing
// else in the system would notice.
//
// The rest is robustness: the plugin is interactive and non-blocking on every
// input, so it must tolerate being processed with nothing connected and being
// handed input events at times the GUI can genuinely produce them.

#include <testing/conformance.h>
#include <testing/fake_host.h>
#include <testing/plugin_under_test.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool cond, const std::string& what) {
    std::printf("  %-58s %s\n", what.c_str(), cond ? "ok" : "FAIL");
    if (!cond) ++failures;
}

bool approx_eq(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

// Column order matches the descriptor: name, type(enum), lat, lon, alt.
SpcListRow make_waypoint(const char* name, int32_t type, double lat, double lon, double alt) {
    SpcListRow r{};
    std::strncpy(r.cells[0].string_val, name, SPC_LIST_CELL_STRING_MAX - 1);
    r.cells[1].enum_val = type;
    r.cells[2].float64_val = lat;
    r.cells[3].float64_val = lon;
    r.cells[4].float64_val = alt;
    return r;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <path-to-adsb_display>\n", argv[0]);
        return 2;
    }

    spc::testing::PluginUnderTest plugin;
    if (!plugin.load(argv[1])) {
        std::fprintf(stderr, "FAIL: %s\n", plugin.error().c_str());
        return 1;
    }
    const auto* d = plugin.descriptor();
    std::printf("plugin: %s\n", d && d->id[0] ? d->id : "<no id>");

    const SpcPluginVTable* vt = plugin.vtable();
    spc::testing::FakeHost host;

    check(plugin.create(), "create_instance");
    if (!plugin.instance()) { std::printf("FAILED: %d\n", ++failures); return 1; }
    plugin.set_host_services(host.services());

    // --- waypoints round-trip ----------------------------------------------
    if (vt->set_list_rows && vt->get_list_rows) {
        std::vector<SpcListRow> in;
        in.push_back(make_waypoint("Heathrow", 1, 51.4700, -0.4543, 83.0));
        in.push_back(make_waypoint("Gatwick", 1, 51.1537, -0.1821, 202.0));
        // A name at the buffer limit: truncation bugs hide behind short strings.
        std::string long_name(SPC_LIST_CELL_STRING_MAX - 1, 'x');
        in.push_back(make_waypoint(long_name.c_str(), 4, -33.9425, 151.1783, 21.0));

        const int set_rc = vt->set_list_rows(plugin.instance(), "waypoints",
                                             in.data(), static_cast<int32_t>(in.size()));
        check(set_rc == 0, "set_list_rows accepts waypoints");

        SpcListRow out[16]{};
        int32_t count = -1;
        const int get_rc = vt->get_list_rows(plugin.instance(), "waypoints", out, 16, &count);
        check(get_rc == 0, "get_list_rows succeeds");
        check(count == static_cast<int32_t>(in.size()),
              "row count survives the round-trip");
        std::printf("    wrote %zu rows, read back %d\n", in.size(), count);

        if (get_rc == 0 && count == static_cast<int32_t>(in.size())) {
            bool names_ok = true, coords_ok = true, enums_ok = true;
            for (size_t i = 0; i < in.size(); ++i) {
                if (std::strcmp(out[i].cells[0].string_val, in[i].cells[0].string_val) != 0)
                    names_ok = false;
                if (!approx_eq(out[i].cells[2].float64_val, in[i].cells[2].float64_val) ||
                    !approx_eq(out[i].cells[3].float64_val, in[i].cells[3].float64_val) ||
                    !approx_eq(out[i].cells[4].float64_val, in[i].cells[4].float64_val))
                    coords_ok = false;
                if (out[i].cells[1].enum_val != in[i].cells[1].enum_val) enums_ok = false;
            }
            check(names_ok, "names survive, including one at the buffer limit");
            // Coordinates are float64 in the schema; a float32 round-trip would
            // move a waypoint by metres.
            check(coords_ok, "lat/lon/alt survive without precision loss");
            check(enums_ok, "waypoint type enum survives");
        }

        // A host asking for fewer rows than exist must not be handed more than
        // it allocated — the classic buffer overrun in this shape of API.
        SpcListRow one_row[1]{};
        int32_t small_count = -1;
        const int trunc_rc = vt->get_list_rows(plugin.instance(), "waypoints",
                                               one_row, 1, &small_count);
        check(small_count <= 1,
              "get_list_rows respects max_rows and never overfills");
        std::printf("    max_rows=1 request returned rc=%d count=%d\n", trunc_rc, small_count);

        // Clearing is what "delete all waypoints" does in the GUI.
        check(vt->set_list_rows(plugin.instance(), "waypoints", nullptr, 0) == 0,
              "set_list_rows accepts an empty list");
        int32_t cleared = -1;
        vt->get_list_rows(plugin.instance(), "waypoints", out, 16, &cleared);
        check(cleared == 0, "cleared list reads back empty");

        // An unknown list name must be refused, not silently treated as one of
        // the real ones.
        int32_t bogus_count = -1;
        check(vt->get_list_rows(plugin.instance(), "__no_such_list__",
                                out, 16, &bogus_count) != 0,
              "get_list_rows rejects an unknown list name");
    } else {
        std::printf("  (no list-row slots — skipping waypoint checks)\n");
    }

    // --- process with nothing connected -------------------------------------
    // Every input is NON_BLOCKING, so the engine will call process() with no
    // data whenever nothing upstream has produced any.
    check(plugin.start() == 0, "start()");
    SpcData outputs[1]{};
    outputs[0].type = SPC_DATA_FRAME;
    const int p_rc = plugin.process(nullptr, 0, outputs, 1);
    check(p_rc == 0 || p_rc < 0, "process() with no inputs returns without faulting");
    std::printf("    process(no inputs) returned %d%s\n", p_rc,
                outputs[0].frame ? " (produced a frame)" : " (no frame)");
    if (outputs[0].frame) {
        check(outputs[0].frame->width > 0 && outputs[0].frame->height > 0,
              "produced map frame has non-zero dimensions");
    }

    // --- input events -------------------------------------------------------
    // The GUI can deliver events before the first frame and after a stop; a
    // plugin that assumes otherwise faults on a stray click.
    // A null event is deliberately not tested: unlike scan_devices, whose ABI
    // comment states host_services may be null, SpcOnInputEventFn documents no
    // such allowance, and the engine calls it as on_input_event(inst, &event)
    // from a reference — so null is unreachable. Asserting on it would invent a
    // contract and fail a plugin that correctly assumes a valid pointer.
    if (vt->on_input_event) {
        SpcInputEvent ev{};
        check(vt->on_input_event(plugin.instance(), &ev) >= -1,
              "on_input_event tolerates a zeroed event");
    }

    check(plugin.stop() == 0, "stop()");
    if (vt->on_input_event) {
        SpcInputEvent ev{};
        check(vt->on_input_event(plugin.instance(), &ev) >= -1,
              "on_input_event after stop is safe");
    }

    plugin.destroy();
    std::printf("%s: %d failure(s)\n", failures ? "FAILED" : "PASSED", failures);
    return failures ? 1 : 0;
}
