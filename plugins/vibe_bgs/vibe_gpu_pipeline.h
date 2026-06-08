#pragma once

#ifdef SPC_HAS_VULKAN

#include <gpu/vulkan_context.h>
#include <gpu/gpu_utils.h>
#include <gpu/gpu_pipeline_base.h>

#include <cstdint>
#include <memory>

namespace spc::gpu {

struct VibePushConstants
{
    int32_t width;
    int32_t height;
    int32_t num_channels;
    int32_t bg_samples;
    int32_t threshold;          // pre-scaled: raw for 8-bit, *256 for 16-bit
    int32_t required_bg_samples;
    int32_t learning_rate_and;
    int32_t bg_samples_and;
    uint32_t frame_number;
    int32_t bytes_per_channel;  // 1 = uint8, 2 = uint16
};

// manages Vulkan compute pipeline and GPU buffers for ViBe BGS
class VibeGpuPipeline : public GpuPipelineBase
{
public:
    VibeGpuPipeline();
    ~VibeGpuPipeline();

    VibeGpuPipeline(const VibeGpuPipeline&) = delete;
    VibeGpuPipeline& operator=(const VibeGpuPipeline&) = delete;

    // initialize GPU resources — returns false on failure
    bool init(VulkanContext& ctx);

    // prepare buffers for given frame dimensions (re-allocates if size changed)
    bool prepare(VulkanContext& ctx, uint32_t width, uint32_t height,
                 int num_channels, int bytes_per_channel, int bg_samples);

    // run initialization pass (first frame), plugin-private cmd buf + submit-and-wait.
    // Use only for the CPU/legacy path where no engine secondary is available.
    // gpu_input: if non-null, device-to-device copy replaces CPU staging upload
    bool run_init(VulkanContext& ctx, const uint8_t* frame_data, uint32_t frame_size,
                  const VibePushConstants& params,
                  VkBuffer gpu_input = VK_NULL_HANDLE);

    // Engine-driven coalesced-submit variant of run_init. Records the
    // init upload + init dispatch into the supplied secondary so the read
    // of gpu_input is properly ordered after upstream writes via the
    // engine's inter-member barrier. Flips model_initialized_ on success;
    // the engine's submit+wait between frames guarantees the GPU work has
    // completed before the next record() call sees the model as ready.
    bool record_init(VulkanContext& ctx, VkCommandBuffer cmd,
                     const uint8_t* frame_data, uint32_t frame_size,
                     const VibePushConstants& params,
                     VkBuffer gpu_input = VK_NULL_HANDLE);

    // Engine-driven coalesced submit (Phase 7). Records the per-frame
    // dispatches into the supplied secondary cmd buffer and returns without
    // submitting. Engine assembles secondaries from a subgraph into one
    // primary and submits once. CPU staging upload of frame_data / detect_mask
    // happens inline (the secondary cmd buffer doesn't change anything about
    // staging — that's plain mapped memory).
    bool record(VulkanContext& ctx, VkCommandBuffer cmd,
                const uint8_t* frame_data, uint32_t frame_size,
                const uint8_t* detect_mask, uint32_t mask_size,
                const VibePushConstants& params,
                VkBuffer gpu_input = VK_NULL_HANDLE);

    VkBuffer input_buffer() const { return input_buf_; }
    VkDeviceSize frame_byte_size() const { return frame_byte_size_; }

    // GPU-resident packed GRAY8 output. Returns whichever buffer the compute
    // just wrote into: in wide layout the pack_mask shader produces
    // packed_mask_buf_; in non-wide layout (today's only active path) the
    // process shader writes packed GRAY8 directly into output_buf_, and
    // packed_mask_buf_ stays empty — returning it would register a zero
    // buffer for GPU-resident downstream consumers.
    VkBuffer packed_mask_buffer() const {
        return use_wide_layout_ ? packed_mask_buf_ : output_buf_;
    }
    VkDeviceMemory packed_mask_memory() const {
        return use_wide_layout_ ? packed_mask_mem_ : output_mem_;
    }
    VkDeviceSize packed_mask_bytes() const { return mask_byte_size_; }
    const void* staging_output_mapped() const { return staging_out_.mapped; }
    const StagingBuffer& output_staging() const { return staging_out_; }
    void invalidate_staging_output(VulkanContext& ctx) { staging_out_.invalidate(ctx); }
    bool model_initialized() const { return model_initialized_; }

    void destroy(VulkanContext& ctx);

    bool initialized() const { return initialized_; }

private:
    // pipelines — packed layout (byte-packed, with atomics — fallback)
    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout desc_layout_ = VK_NULL_HANDLE;
    VkDescriptorPool desc_pool_ = VK_NULL_HANDLE;
    VkDescriptorSet desc_set_ = VK_NULL_HANDLE;
    VkPipeline init_pipeline_ = VK_NULL_HANDLE;
    VkPipeline process_pipeline_ = VK_NULL_HANDLE;

    // pipelines — wide layout (uint32 per pixel, no atomics — preferred)
    VkPipeline init_wide_pipeline_ = VK_NULL_HANDLE;
    VkPipeline process_wide_pipeline_ = VK_NULL_HANDLE;
    VkPipeline pack_mask_pipeline_ = VK_NULL_HANDLE;  // wide uint32 → packed GRAY8

    // shader objects (VK_EXT_shader_object)
    VkShaderEXT init_shader_ = VK_NULL_HANDLE;
    VkShaderEXT process_shader_ = VK_NULL_HANDLE;
    VkShaderEXT init_wide_shader_ = VK_NULL_HANDLE;
    VkShaderEXT process_wide_shader_ = VK_NULL_HANDLE;
    VkShaderEXT pack_mask_shader_ = VK_NULL_HANDLE;

    // GPU buffers (device-local)
    VkBuffer input_buf_ = VK_NULL_HANDLE;
    VkDeviceMemory input_mem_ = VK_NULL_HANDLE;
    VkBuffer bg_model_buf_ = VK_NULL_HANDLE;
    VkDeviceMemory bg_model_mem_ = VK_NULL_HANDLE;
    VkBuffer output_buf_ = VK_NULL_HANDLE;
    VkDeviceMemory output_mem_ = VK_NULL_HANDLE;
    VkBuffer detect_mask_buf_ = VK_NULL_HANDLE;
    VkDeviceMemory detect_mask_mem_ = VK_NULL_HANDLE;
    VkBuffer packed_mask_buf_ = VK_NULL_HANDLE;   // packed GRAY8 output (binding 4)
    VkDeviceMemory packed_mask_mem_ = VK_NULL_HANDLE;

    // push descriptor buffer cache
    VkBuffer push_bufs_[5] = {};
    VkDeviceSize push_sizes_[5] = {};

    // staging buffers (host-visible, persistently mapped)
    StagingBuffer staging_in_;
    StagingBuffer staging_out_;
    StagingBuffer staging_mask_;

    // cached dimensions and layout
    uint32_t width_ = 0;
    uint32_t height_ = 0;
    int num_channels_ = 0;
    int bytes_per_channel_ = 0;
    int bg_samples_ = 0;
    uint32_t pixel_count_ = 0;
    bool use_wide_layout_ = false;
    bool use_bar_input_ = false;     // true if input_buf_ is HOST_VISIBLE+DEVICE_LOCAL (ReBAR)
    bool initialized_ = false;
    bool model_initialized_ = false;

    // cached buffer sizes
    VkDeviceSize frame_byte_size_ = 0;   // input frame in bytes
    VkDeviceSize mask_byte_size_ = 0;    // detection mask in bytes
    VkDeviceSize output_buf_size_ = 0;   // output device buffer size (wide or packed)
    VkDeviceSize staging_out_size_ = 0;  // output staging buffer size
};

} // namespace spc::gpu

#endif // SPC_HAS_VULKAN
