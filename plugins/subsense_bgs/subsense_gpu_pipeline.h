#pragma once

#ifdef SPC_HAS_VULKAN

#include <gpu/vulkan_context.h>
#include <gpu/gpu_utils.h>
#include <gpu/gpu_pipeline_base.h>

#include <cstdint>

namespace spc::gpu {

struct SubsensePushConstants
{
    int32_t width;
    int32_t height;
    int32_t num_channels;       // 1 or 3
    int32_t bg_samples;
    int32_t color_threshold;    // color distance threshold
    int32_t desc_threshold;     // LBSP hamming distance threshold
    int32_t required_matches;   // min matches to classify as background
    int32_t learning_rate;      // 1/N update probability (e.g. 16 = 1/16)
    uint32_t frame_number;      // frame counter
    int32_t has_mask;           // 1 if detection mask is provided
    int32_t width4;             // ceil(width/4)
    int32_t pad0;
};

// manages Vulkan compute pipelines and GPU buffers for SuBSENSE BGS
class SubsenseGpuPipeline : public GpuPipelineBase
{
public:
    SubsenseGpuPipeline();
    ~SubsenseGpuPipeline();

    SubsenseGpuPipeline(const SubsenseGpuPipeline&) = delete;
    SubsenseGpuPipeline& operator=(const SubsenseGpuPipeline&) = delete;

    // initialize GPU resources (pipelines, descriptors)
    bool init(VulkanContext& ctx);

    // prepare buffers for given frame dimensions (re-allocates if changed)
    bool prepare(VulkanContext& ctx, uint32_t width, uint32_t height,
                 int num_channels, int bg_samples);

    // Engine-driven coalesced-submit init pass (first frame — fills background
    // model). Records upload + barrier + init dispatch into the supplied
    // secondary so the read of gpu_input is properly ordered after upstream
    // writes via the engine's inter-member barrier. Flips model_initialized_
    // on success.
    //
    // in_device / in_staging are the engine-owned INPUT upload ring slot (the
    // plugin already memcpy'd the CPU frame into in_staging's mapped memory);
    // out_device is the engine-owned OUTPUT ring slot bound to binding 4 so the
    // descriptor set is fully valid. When gpu_input != NULL the upstream output
    // is read device-to-device and in_device / in_staging are ignored.
    bool record_init(VulkanContext& ctx, VkCommandBuffer cmd,
                     VkBuffer in_device, VkBuffer in_staging,
                     VkBuffer out_device,
                     const SubsensePushConstants& params,
                     VkBuffer gpu_input = VK_NULL_HANDLE);

    // Engine-driven coalesced submit (Phase 7). Records the per-frame
    // process dispatches into the supplied secondary cmd buffer and returns
    // without submitting. Caller must guarantee model_ready() is true.
    //
    // Both the INPUT upload buffer and the OUTPUT buffer are engine-owned
    // K-deep ring slots resolved by the plugin from the host (so frame N's
    // buffers don't clobber a still-in-flight frame's). The plugin already
    // memcpy'd the CPU frame into in_staging's mapped memory.
    //   in_device / in_staging — upload ring slot (binding 0 + upload source);
    //                            ignored when gpu_input != NULL.
    //   out_device — output ring slot (binding 4).
    //   detect_mask — optional, uploaded via the pipeline's own staging_mask_.
    bool record(VulkanContext& ctx, VkCommandBuffer cmd,
                VkBuffer in_device, VkBuffer in_staging,
                VkBuffer out_device,
                const uint8_t* detect_mask, int mask_stride,
                const SubsensePushConstants& params,
                VkBuffer gpu_input = VK_NULL_HANDLE);

    // Byte sizes the plugin passes to the host's edge-ring acquire calls.
    VkDeviceSize input_device_size() const { return frame_bytes_; }
    VkDeviceSize output_device_size() const { return mask_bytes_; }
    VkDeviceSize output_staging_size() const { return mask_bytes_; }
    VkDeviceSize packed_mask_bytes() const { return mask_bytes_; }

    void destroy(VulkanContext& ctx);

    bool initialized() const { return initialized_; }
    bool model_ready() const { return model_initialized_; }

private:
    // Re-point the per-frame ring bindings 0 (input) and 4 (output). `out` ==
    // VK_NULL_HANDLE leaves binding 4 untouched (the init pass doesn't write
    // the output). Updates the push-descriptor cache, or writes the classic
    // descriptor set when push descriptors are unavailable.
    void set_ring_bindings(VulkanContext& ctx, VkBuffer in, VkBuffer out);

    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout desc_layout_ = VK_NULL_HANDLE;
    VkDescriptorPool desc_pool_ = VK_NULL_HANDLE;
    VkDescriptorSet desc_set_ = VK_NULL_HANDLE;
    VkPipeline init_pipeline_ = VK_NULL_HANDLE;
    VkPipeline process_pipeline_ = VK_NULL_HANDLE;
    VkPipeline pack_mask_pipeline_ = VK_NULL_HANDLE;  // wide uint32 -> packed GRAY8
    VkShaderEXT init_shader_ = VK_NULL_HANDLE;
    VkShaderEXT process_shader_ = VK_NULL_HANDLE;
    VkShaderEXT pack_mask_shader_ = VK_NULL_HANDLE;

    // 5 buffers: input, bg_colors, bg_descs, detect_mask, output.
    // bufs_[0] (input, binding 0) and bufs_[4] (output, binding 4) are NO
    // LONGER owned here — they are engine-owned K-deep ring slots resolved
    // per-frame from the host (acquire_ringed_upload / acquire_ringed_output).
    // At K=1 each is a single slot, byte-identical to the prior plugin-owned
    // single buffer. bg_colors/bg_descs are genuine cross-frame STATE and
    // detect_mask is a within-frame side input — all stay single.
    static constexpr int NUM_BUFFERS = 5;
    VkBuffer bufs_[NUM_BUFFERS]{};
    VkDeviceMemory mems_[NUM_BUFFERS]{};

    // packed GRAY8 output buffer (binding 5) — bound for descriptor validity
    // (only written by the disabled wide-layout pack_mask shader).
    VkBuffer packed_mask_buf_ = VK_NULL_HANDLE;
    VkDeviceMemory packed_mask_mem_ = VK_NULL_HANDLE;

    // push descriptor buffer cache. Bindings 0 (input) and 4 (output) are
    // engine-ring slots re-pointed per frame; the rest are stable.
    VkBuffer push_bufs_[NUM_BUFFERS + 1] = {};
    VkDeviceSize push_sizes_[NUM_BUFFERS + 1] = {};

    // staging buffer for the optional detect_mask (host-visible, mapped). The
    // input upload + output download staging are now engine-owned ring slots.
    StagingBuffer staging_mask_;

    // cached dimensions
    uint32_t width_ = 0;
    uint32_t height_ = 0;
    int num_channels_ = 0;
    int bg_samples_ = 0;
    VkDeviceSize frame_bytes_ = 0;      // input frame size (aligned)
    VkDeviceSize mask_bytes_ = 0;       // detection mask / output mask
    VkDeviceSize bg_color_bytes_ = 0;   // bg color model size
    VkDeviceSize bg_desc_bytes_ = 0;    // bg descriptor model size
    uint32_t pixel_count_ = 0;
    bool use_wide_layout_ = false;
    bool initialized_ = false;
    bool model_initialized_ = false;
};

} // namespace spc::gpu

#endif // SPC_HAS_VULKAN
