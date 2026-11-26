#pragma once
// Drives a drawing plugin's record_gpu() on the real device as the engine would
// for one frame of a node with no input frame: the rings handed out from the
// harness's own host-visible buffers, the command buffer submitted and waited
// on, the output read back.

// the harness includes windows.h
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <testing/plugin_under_test.h>

#include <gpu/vulkan_context.h>

#include <speculor/plugin.h>

#include <cstdint>
#include <vector>

namespace display_test {

// RGB24 rows of w * 3 bytes
struct Image {
    uint32_t w = 0, h = 0;
    std::vector<uint8_t> px;
};

// A frame process() made; false unless it is RGB24.
bool read_frame(const SpcFrame* f, Image& out);

// record_gpu() over `inputs`, its output frame read back.
bool record_gpu(spc::gpu::VulkanContext& ctx, spc::testing::PluginUnderTest& p, SpcData* inputs,
                uint32_t input_count, Image& out);

// share of the pixels whose channels differ by more than `tol`; 1 for different sizes
double share_differing(const Image& a, const Image& b, int tol);

}  // namespace display_test
