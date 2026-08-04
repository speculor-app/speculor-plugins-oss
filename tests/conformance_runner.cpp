// Runs the SDK's plugin conformance checks against one built plugin, named on
// the command line, and exits non-zero on any failure.
//
// A bare main rather than a test framework: the harness is framework-agnostic
// by design, and this repo builds against a self-contained SDK bundle with no
// other dependencies — pulling in Catch2 or GoogleTest just to call one
// function and compare a bool would undo that. CMake registers one ctest entry
// per plugin, so a failure still names the plugin that broke.

#include <testing/conformance.h>
#include <testing/plugin_under_test.h>

#include <cstdio>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <path-to-plugin-module>\n", argv[0]);
        return 2;
    }
    const char* path = argv[1];

    spc::testing::PluginUnderTest plugin;
    if (!plugin.load(path)) {
        std::fprintf(stderr, "FAIL: %s\n", plugin.error().c_str());
        return 1;
    }

    const auto* d = plugin.descriptor();
    std::printf("plugin: %s (%s)\n", d && d->id[0] ? d->id : "<no id>", path);

    const auto report = spc::testing::run_conformance(plugin);
    std::printf("%s\n", report.summary().c_str());
    return report.ok() ? 0 : 1;
}
