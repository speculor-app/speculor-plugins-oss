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

    // Engine-driven coalesced first-frame init. Records the init upload +
    // init dispatch into the supplied secondary so the read of the input is
    // ordered after upstream writes via the engine's inter-member barrier.
    //
    // The input buffer is engine-owned (K-deep upload ring): the plugin
    // already memcpy'd the CPU frame into `in_staging`'s mapped memory before
    // calling this. in_device is the upload ring slot's device buffer (the
    // init dispatch reads binding 0 from it); in_staging is that slot's
    // host-visible staging (upload source). When gpu_input != NULL the
    // upstream output is read directly (device-to-device) and in_device /
    // in_staging are ignored. Flips model_initialized_ on success.
    // out_device is the output ring slot's device buffer — bound to binding 2
    // so the descriptor set is fully valid even though the init shader writes
    // only bg_model (binding 1), not the output. Must be non-null and sized
    // output_device_size().
    bool record_init(VulkanContext& ctx, VkCommandBuffer cmd,
                     VkBuffer in_device, VkBuffer in_staging, uint32_t frame_size,
                     VkBuffer out_device,
                     const VibePushConstants& params,
                     VkBuffer gpu_input = VK_NULL_HANDLE);

    // Engine-driven coalesced submit (Phase 7). Records the per-frame
    // dispatches into the supplied secondary cmd buffer and returns without
    // submitting. Both the INPUT upload buffer and the OUTPUT buffer are
    // engine-owned K-deep ring slots resolved by the plugin from the host
    // (so frame N's buffers don't clobber a still-in-flight frame's). The
    // plugin already memcpy'd the CPU frame into in_staging's mapped memory.
    //   in_device / in_staging — upload ring slot (binding 0 + upload source);
    //                            ignored when gpu_input != NULL.
    //   out_device — output ring slot (binding 2).
    //   detect_mask — optional, uploaded via the pipeline's own staging_mask_.
    bool record(VulkanContext& ctx, VkCommandBuffer cmd,
                VkBuffer in_device, VkBuffer in_staging, uint32_t frame_size,
                VkBuffer out_device,
                const uint8_t* detect_mask, uint32_t mask_size,
                const VibePushConstants& params,
                VkBuffer gpu_input = VK_NULL_HANDLE);

    // Byte sizes the plugin passes to the host's edge-ring acquire calls.
    VkDeviceSize input_device_size() const { return frame_byte_size_; }
    VkDeviceSize output_device_size() const { return output_buf_size_; }
    VkDeviceSize output_staging_size() const { return staging_out_size_; }
    VkDeviceSize packed_mask_bytes() const { return mask_byte_size_; }
    bool model_initialized() const { return model_initialized_; }

    // Mark the (single, persistent) detect-mask device buffer as needing a
    // re-upload. The plugin calls this only when a new mask is actually cached;
    // record() uploads once and clears it, instead of re-pushing the identical
    // mask every frame. prepare() also sets it on a resolution change.
    void mark_mask_dirty() { mask_upload_needed_ = true; }

    void destroy(VulkanContext& ctx);

    bool initialized() const { return initialized_; }

private:
    // Re-point the per-frame ring bindings 0 (input) and 2 (output). `out` ==
    // VK_NULL_HANDLE leaves binding 2 untouched (the init pass doesn't write
    // the output). Updates the push-descriptor cache, or writes the classic
    // descriptor set when push descriptors are unavailable.
    void set_ring_bindings(VulkanContext& ctx, VkBuffer in, VkBuffer out);

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

    // GPU buffers (device-local). bg_model/detect_mask stay SINGLE: bg_model
    // is genuine cross-frame state, detect_mask is an optional NON_BLOCKING
    // side input written-then-read within one frame; the single compute queue
    // serializes frames in submission order so neither needs ringing.
    //
    // The INPUT upload buffer (binding 0) and the OUTPUT buffer (binding 2)
    // are NO LONGER owned here — they are engine-owned K-deep ring slots
    // resolved per-frame from the host (acquire_ringed_upload /
    // acquire_ringed_output). At K=1 each is a single slot, byte-identical to
    // the prior plugin-owned single buffer.
    VkBuffer bg_model_buf_ = VK_NULL_HANDLE;
    VkDeviceMemory bg_model_mem_ = VK_NULL_HANDLE;
    VkBuffer detect_mask_buf_ = VK_NULL_HANDLE;
    VkDeviceMemory detect_mask_mem_ = VK_NULL_HANDLE;
    VkBuffer packed_mask_buf_ = VK_NULL_HANDLE;   // packed GRAY8 output (binding 4, wide layout only)
    VkDeviceMemory packed_mask_mem_ = VK_NULL_HANDLE;

    // push descriptor buffer cache. Bindings 0 (input) and 2 (output) are
    // engine-ring slots re-pointed at the top of every record()/record_init();
    // bindings 1/3/4 are the stable plugin-owned buffers set in prepare().
    VkBuffer push_bufs_[5] = {};
    VkDeviceSize push_sizes_[5] = {};

    // staging buffer for the optional detect_mask (host-visible, mapped). The
    // input upload staging is now engine-owned (the upload ring slot).
    StagingBuffer staging_mask_;

    // cached dimensions and layout
    uint32_t width_ = 0;
    uint32_t height_ = 0;
    int num_channels_ = 0;
    int bytes_per_channel_ = 0;
    int bg_samples_ = 0;
    uint32_t pixel_count_ = 0;
    bool use_wide_layout_ = false;
    bool initialized_ = false;
    bool model_initialized_ = false;
    bool mask_upload_needed_ = false;  // detect mask changed → re-upload once

    // cached buffer sizes
    VkDeviceSize frame_byte_size_ = 0;   // input frame in bytes
    VkDeviceSize mask_byte_size_ = 0;    // detection mask in bytes
    VkDeviceSize output_buf_size_ = 0;   // output device buffer size (wide or packed)
    VkDeviceSize staging_out_size_ = 0;  // output staging buffer size
};

} // namespace spc::gpu

#endif // SPC_HAS_VULKAN
