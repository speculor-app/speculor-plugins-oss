// The SDR source plugins with no radio attached.
//
// CI has no dongle, so streaming cannot be tested — but the paths that have
// actually caused trouble do not need one. A device that disappears mid-stream,
// a scan that finds nothing, a stop that races a start: those run the same code
// whether or not hardware is present, and they are where the heap corruption in
// librtlsdr and the unbounded-shutdown bugs in SAPIENT both lived.
//
// Listing devices is the interesting one here: it runs with no instance, as
// often as the user asks, and rewrites the plugin's own device registry. The
// full conformance suite is run again after it.

#include <testing/conformance.h>
#include <testing/fake_host.h>
#include <testing/plugin_under_test.h>

#include <cstdio>
#include <memory>
#include <cstring>
#include <string>

namespace {

int failures = 0;

void check(bool cond, const std::string& what) {
    std::printf("  %-58s %s\n", what.c_str(), cond ? "ok" : "FAIL");
    if (!cond) ++failures;
}

const SpcParameterDesc* find_param(const SpcPluginDescriptor* d, const char* name) {
    if (!d) return nullptr;
    for (uint32_t i = 0; i < d->param_count && i < SPC_PLUGIN_PARAM_MAX; ++i) {
        if (std::strcmp(d->params[i].name, name) == 0) return &d->params[i];
    }
    return nullptr;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <path-to-sdr-plugin>\n", argv[0]);
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

    // --- scanning with nothing attached -------------------------------------
    if (vt->list_options) {
        // The ABI lets a host pass null services; a plugin that assumes
        // otherwise faults during device enumeration in the GUI.
        spc::ParamHost values(vt, d);
        values.list_options(nullptr, true);
        values.list_options(host.services(), true);

        // Repeated scans are normal: the GUI rescans whenever the user asks.
        for (int i = 0; i < 5; ++i) values.list_options(host.services(), true);

        auto dev = std::make_unique<SpcParameterDesc>();
        if (values.get("device", *dev) == 0) {
            check(dev->enum_val.count >= 0 && dev->enum_val.count <= SPC_PARAM_ENUM_MAX,
                  "device enum count stays within SPC_PARAM_ENUM_MAX");
            check(dev->enum_val.count == 0 ||
                      (dev->enum_val.value >= 0 && dev->enum_val.value < dev->enum_val.count),
                  "device selection stays within the listed options");
        }

        // The whole descriptor, not just the enum: scanning must not leave any
        // part of it malformed.
        const auto post = spc::testing::run_conformance(plugin);
        check(post.ok(), "descriptor still passes conformance after scanning");
        if (!post.ok()) std::printf("%s\n", post.summary().c_str());
    } else {
        std::printf("  (no list_options slot — skipping scan checks)\n");
    }

    // --- lifecycle with no device selected ----------------------------------
    check(plugin.create(), "create_instance");
    plugin.set_host_services(host.services());

    // Nothing is attached and no device is chosen, so start() should report
    // success and simply produce nothing — not fail, and not open a radio.
    const int rc_start = plugin.start();
    check(rc_start == 0, "start() with no device selected succeeds");
    std::printf("    start() returned %d\n", rc_start);

    // Two-phase shutdown: request_stop is the abort signal delivered while
    // process() may still be running, stop is the teardown after it returns.
    check(plugin.request_stop() == 0, "request_stop() after start");
    check(plugin.stop() == 0, "stop() after request_stop");

    // The engine legitimately stops a node that already stopped itself after a
    // fatal error, and three separate hangs this month came out of that path.
    check(plugin.stop() == 0, "stop() is idempotent");
    check(plugin.request_stop() == 0, "request_stop() after stop is safe");

    plugin.destroy();
    check(true, "destroy after stop");

    // --- stop without start -------------------------------------------------
    check(plugin.create(), "create_instance (second)");
    plugin.set_host_services(host.services());
    check(plugin.stop() == 0, "stop() without a preceding start is safe");
    plugin.destroy();

    // --- restart ------------------------------------------------------------
    // Restarting a source is ordinary engine behaviour after a parameter edit.
    check(plugin.create(), "create_instance (third)");
    plugin.set_host_services(host.services());
    check(plugin.start() == 0, "start() again on a fresh instance");
    check(plugin.stop() == 0, "stop() again");
    check(plugin.start() == 0, "start() after stop on the same instance");
    check(plugin.stop() == 0, "final stop()");
    plugin.destroy();

    std::printf("%s: %d failure(s)\n", failures ? "FAILED" : "PASSED", failures);
    return failures ? 1 : 0;
}
