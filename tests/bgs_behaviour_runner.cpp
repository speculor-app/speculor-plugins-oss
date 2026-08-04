// Behavioural checks for the background-subtraction plugins: feed synthetic
// frames through the C ABI and assert what the algorithm is supposed to do —
// a settled scene reports almost no foreground, and an object appearing in it
// is reported where it actually is.
//
// Assertions are invariants, not exact pixels. ViBe replaces a randomly chosen
// background sample per update by design, so two runs over identical input do
// not have to agree pixel-for-pixel; a test demanding that would fail for a
// correct implementation. Which plugin to drive is argv[1], so one runner
// serves both.

#include <testing/fake_host.h>
#include <testing/plugin_under_test.h>

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

constexpr uint32_t kW = 160;
constexpr uint32_t kH = 120;

int failures = 0;

void check(bool cond, const std::string& what) {
    std::printf("  %-58s %s\n", what.c_str(), cond ? "ok" : "FAIL");
    if (!cond) ++failures;
}

// Mid-grey canvas; a flat scene the model can settle on quickly.
std::vector<uint8_t> make_scene(uint8_t level = 120) {
    return std::vector<uint8_t>(static_cast<size_t>(kW) * kH, level);
}

// Bright square, deliberately far from the borders so a mask that is merely
// noisy at the edges cannot be mistaken for a detection.
void draw_blob(std::vector<uint8_t>& px, uint32_t x0, uint32_t y0, uint32_t size,
               uint8_t level = 250) {
    for (uint32_t y = y0; y < y0 + size && y < kH; ++y) {
        for (uint32_t x = x0; x < x0 + size && x < kW; ++x) {
            px[static_cast<size_t>(y) * kW + x] = level;
        }
    }
}

size_t count_nonzero(const SpcFrame* f) {
    if (!f || !f->data) return 0;
    size_t n = 0;
    for (uint32_t y = 0; y < f->height; ++y) {
        const uint8_t* row = f->data + static_cast<size_t>(y) * f->stride;
        for (uint32_t x = 0; x < f->width; ++x) {
            if (row[x]) ++n;
        }
    }
    return n;
}

// Foreground pixels inside the blob's rectangle, so "detected in the right
// place" is distinguishable from "the whole mask lit up".
size_t count_nonzero_in(const SpcFrame* f, uint32_t x0, uint32_t y0, uint32_t size) {
    if (!f || !f->data) return 0;
    size_t n = 0;
    for (uint32_t y = y0; y < y0 + size && y < f->height; ++y) {
        const uint8_t* row = f->data + static_cast<size_t>(y) * f->stride;
        for (uint32_t x = x0; x < x0 + size && x < f->width; ++x) {
            if (row[x]) ++n;
        }
    }
    return n;
}

// One process() call with `px` as the input image. Returns the mask the plugin
// wrote to output 0, or nullptr if it produced none.
const SpcFrame* run_frame(spc::testing::PluginUnderTest& p,
                          spc::testing::FakeHost& host,
                          std::vector<uint8_t>& px, uint64_t frame_no) {
    SpcFrame in{};
    in.data = px.data();
    in.width = kW;
    in.height = kH;
    in.stride = kW;
    in.format = SPC_PIXEL_FORMAT_GRAY8;
    in.frame_number = frame_no;
    in.timestamp_ns = static_cast<int64_t>(frame_no) * 33'000'000;

    SpcData input{};
    input.type = SPC_DATA_FRAME;
    input.frame = &in;

    // Two outputs: the descriptor declares mask_out then image_out, and
    // process() rejects anything narrower.
    SpcData outputs[2]{};
    outputs[0].type = SPC_DATA_FRAME;
    outputs[1].type = SPC_DATA_FRAME;

    (void)host;
    if (p.process(&input, 1, outputs, 2) != 0) return nullptr;
    return outputs[0].frame;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <path-to-bgs-plugin>\n", argv[0]);
        return 2;
    }

    spc::testing::PluginUnderTest plugin;
    if (!plugin.load(argv[1])) {
        std::fprintf(stderr, "FAIL: %s\n", plugin.error().c_str());
        return 1;
    }
    const auto* d = plugin.descriptor();
    std::printf("plugin: %s\n", d && d->id[0] ? d->id : "<no id>");

    spc::testing::FakeHost host;
    if (!plugin.create()) { std::fprintf(stderr, "FAIL: create_instance\n"); return 1; }
    plugin.set_host_services(host.services());
    if (plugin.start() != 0) { std::fprintf(stderr, "FAIL: start\n"); return 1; }

    // --- settle the model on a static scene ---------------------------------
    auto scene = make_scene();
    const SpcFrame* mask = nullptr;
    for (uint64_t i = 0; i < 60; ++i) {
        mask = run_frame(plugin, host, scene, i);
    }
    check(mask != nullptr, "static scene: plugin produced a mask");
    if (!mask) { plugin.destroy(); return 1; }

    check(mask->width == kW && mask->height == kH,
          "static scene: mask matches input dimensions");

    const size_t total = static_cast<size_t>(kW) * kH;
    const size_t settled_fg = count_nonzero(mask);
    std::printf("    settled foreground: %zu / %zu px\n", settled_fg, total);
    // An unchanging scene is background by definition. A few stray pixels are
    // tolerable; a large fraction means the model never converged.
    check(settled_fg < total / 20, "static scene: <5% of pixels report foreground");

    // --- an object appears --------------------------------------------------
    constexpr uint32_t bx = 60, by = 40, bsz = 30;
    auto with_blob = make_scene();
    draw_blob(with_blob, bx, by, bsz);
    mask = run_frame(plugin, host, with_blob, 100);
    check(mask != nullptr, "blob frame: plugin produced a mask");
    if (!mask) { plugin.destroy(); return 1; }

    const size_t in_blob = count_nonzero_in(mask, bx, by, bsz);
    const size_t blob_area = static_cast<size_t>(bsz) * bsz;
    const size_t all_fg = count_nonzero(mask);
    std::printf("    foreground in blob: %zu / %zu px (mask total %zu)\n",
                in_blob, blob_area, all_fg);
    check(in_blob > blob_area / 2, "blob frame: majority of the blob reads foreground");
    // Guards against a mask that simply saturates: the detection has to be
    // localised, not the whole frame lighting up.
    check(all_fg < total / 2, "blob frame: detection is localised, not whole-frame");

    // --- the object stays put and is absorbed -------------------------------
    // ViBe deliberately absorbs a stationary object over time; the point is
    // that foreground shrinks rather than persisting forever.
    for (uint64_t i = 0; i < 120; ++i) {
        mask = run_frame(plugin, host, with_blob, 200 + i);
    }
    const size_t absorbed = mask ? count_nonzero_in(mask, bx, by, bsz) : 0;
    std::printf("    foreground in blob after 120 static frames: %zu px\n", absorbed);
    check(absorbed <= in_blob, "stationary object: foreground does not grow over time");

    // --- shutdown -----------------------------------------------------------
    check(plugin.stop() == 0, "stop() returns success");
    plugin.destroy();

    // Frames the plugin took from the host must come back, or a long-running
    // pipeline leaks its pool.
    std::printf("    host frames: %zu acquired, %zu released\n",
                host.acquired(), host.released());

    std::printf("%s: %d failure(s)\n", failures ? "FAILED" : "PASSED", failures);
    return failures ? 1 : 0;
}
