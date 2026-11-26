#include "display_gpu_harness.h"

#include <gpu/gpu_utils.h>

#include <cstdlib>
#include <cstring>
#include <map>
#include <utility>

using spc::gpu::VulkanContext;

namespace display_test {

namespace {

constexpr uint64_t kOutputHandle = 77;

// What the engine lends a recording node, reduced to one frame.
struct Pass : SpcGpuPass {
    VulkanContext* ctx;
    struct Buf {
        VkBuffer b = VK_NULL_HANDLE;
        VkDeviceMemory m = VK_NULL_HANDLE;
        void* map = nullptr;
    };
    std::vector<Buf> bufs;
    std::map<uint32_t, spc::gpu::StagingBuffer> scratch;
    std::vector<std::pair<void (*)(void*), void*>> releases;
    Buf output;

    explicit Pass(VulkanContext& c) : SpcGpuPass{}, ctx(&c) {
        acquire_ringed_output = [](SpcGpuPass* p, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t,
                                   uint64_t device_size, uint64_t, SpcGpuEdgeBuffer* out) {
            auto* self = static_cast<Pass*>(p);
            const Buf b = self->host_buffer(device_size);
            if (!b.b) return -1;
            self->output = b;
            *out = {};
            out->device_buffer = b.b;
            out->device_memory = b.m;
            out->gpu_handle = kOutputHandle;
            return 0;
        };
        acquire_ringed_scratch = [](SpcGpuPass* p, uint32_t key, uint32_t, uint64_t size, SpcGpuEdgeBuffer* out) {
            auto* self = static_cast<Pass*>(p);
            auto& s = self->scratch[key];
            if (s.size < size) {
                s.destroy(*self->ctx);
                if (!s.allocate(*self->ctx, size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT)) return -1;
            }
            *out = {};
            out->staging_buffer = s.buffer;
            out->staging_mapped = s.mapped;
            out->staging_size = s.size;
            return 0;
        };
        release_after_frame = [](SpcGpuPass* p, void (*release)(void*), void* arg) {
            static_cast<Pass*>(p)->releases.emplace_back(release, arg);
        };
    }

    Buf host_buffer(uint64_t size) {
        Buf b;
        if (!spc::gpu::create_buffer(*ctx, size, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                     VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
                                                     VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                     VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                                     b.b, b.m) ||
            vkMapMemory(ctx->device, b.m, 0, VK_WHOLE_SIZE, 0, &b.map) != VK_SUCCESS)
            return {};
        bufs.push_back(b);
        return b;
    }

    ~Pass() {
        for (auto& [fn, arg] : releases) fn(arg);
        for (auto& b : bufs) {
            vkUnmapMemory(ctx->device, b.m);
            spc::gpu::destroy_buffer(*ctx, b.b, b.m);
        }
        for (auto& [k, s] : scratch) s.destroy(*ctx);
    }
};

bool submit(VulkanContext& ctx, VkCommandBuffer cmd) {
    VkFence fence = VK_NULL_HANDLE;
    VkFenceCreateInfo fi{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    if (vkCreateFence(ctx.device, &fi, nullptr, &fence) != VK_SUCCESS) return false;
    VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    si.commandBufferCount = 1;
    si.pCommandBuffers = &cmd;
    ctx.lock_queue_submit(ctx.compute_queue_family);
    VkResult r = vkQueueSubmit(ctx.compute_queue, 1, &si, fence);
    ctx.unlock_queue_submit(ctx.compute_queue_family);
    if (r == VK_SUCCESS) r = vkWaitForFences(ctx.device, 1, &fence, VK_TRUE, 5'000'000'000ULL);
    vkDestroyFence(ctx.device, fence, nullptr);
    return r == VK_SUCCESS;
}

}  // namespace

bool read_frame(const SpcFrame* f, Image& out) {
    if (!f || !f->data || f->format != SPC_PIXEL_FORMAT_RGB24) return false;
    out = {f->width, f->height, {}};
    out.px.resize(static_cast<size_t>(f->width) * f->height * 3);
    for (uint32_t y = 0; y < f->height; ++y)
        std::memcpy(&out.px[static_cast<size_t>(y) * f->width * 3], f->data + static_cast<size_t>(y) * f->stride,
                    static_cast<size_t>(f->width) * 3);
    return true;
}

bool record_gpu(VulkanContext& ctx, spc::testing::PluginUnderTest& p, SpcData* inputs, uint32_t input_count,
                Image& out) {
    if (!p.vtable()->record_gpu) return false;
    Pass pass(ctx);
    SpcData outputs[1]{};
    SpcStep step{inputs, input_count, outputs, 1};

    VkCommandPool pool = VK_NULL_HANDLE;
    VkCommandPoolCreateInfo ci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    ci.queueFamilyIndex = ctx.compute_queue_family;
    bool ok = vkCreateCommandPool(ctx.device, &ci, nullptr, &pool) == VK_SUCCESS;
    VkCommandBuffer cmd = VK_NULL_HANDLE;
    VkCommandBufferAllocateInfo ai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    ai.commandPool = pool;
    ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    ai.commandBufferCount = 1;
    ok = ok && vkAllocateCommandBuffers(ctx.device, &ai, &cmd) == VK_SUCCESS;
    VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    ok = ok && vkBeginCommandBuffer(cmd, &bi) == VK_SUCCESS;
    pass.cmd_buffer_handle = cmd;
    ok = ok && p.vtable()->record_gpu(p.instance(), &step, &pass) == 0;
    ok = ok && vkEndCommandBuffer(cmd) == VK_SUCCESS;
    for (auto& [k, s] : pass.scratch) s.flush(ctx);
    ok = ok && submit(ctx, cmd);

    const SpcFrame* f = outputs[0].frame;
    ok = ok && f && f->gpu_handle == kOutputHandle && (f->gpu_flags & SPC_GPU_FLAG_RESIDENT) &&
         f->format == SPC_PIXEL_FORMAT_RGB24 && pass.output.map;
    if (ok) {
        const uint32_t stride = (f->width + 3) / 4 * 12;
        out = {f->width, f->height, {}};
        out.px.resize(static_cast<size_t>(f->width) * f->height * 3);
        for (uint32_t y = 0; y < f->height; ++y)
            std::memcpy(&out.px[static_cast<size_t>(y) * f->width * 3],
                        static_cast<const uint8_t*>(pass.output.map) + static_cast<size_t>(y) * stride,
                        static_cast<size_t>(f->width) * 3);
    }
    if (pool) vkDestroyCommandPool(ctx.device, pool, nullptr);
    return ok;
}

double share_differing(const Image& a, const Image& b, int tol) {
    if (a.w != b.w || a.h != b.h || a.px.size() != b.px.size()) return 1.0;
    size_t bad = 0;
    for (size_t i = 0; i < a.px.size(); i += 3)
        for (size_t c = 0; c < 3; ++c)
            if (std::abs(int(a.px[i + c]) - int(b.px[i + c])) > tol) {
                ++bad;
                break;
            }
    return static_cast<double>(bad) / static_cast<double>(a.px.size() / 3);
}

}  // namespace display_test
