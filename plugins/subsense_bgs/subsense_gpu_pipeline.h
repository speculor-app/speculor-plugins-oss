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

    // run initialization pass (first frame — fills background model).
    // Plugin-private cmd buf + submit_and_wait — use only for the CPU/legacy
    // path where no engine secondary is available.
    bool run_init(VulkanContext& ctx, const uint8_t* frame_data, int frame_stride,
                  const SubsensePushConstants& params,
                  VkBuffer gpu_input = VK_NULL_HANDLE);

    // Engine-driven coalesced-submit variant of run_init. Records upload +
    // barrier + init dispatch into the supplied secondary so the read of
    // gpu_input is properly ordered after upstream writes via the engine's
    // inter-member barrier. Flips model_initialized_ on success.
    bool record_init(VulkanContext& ctx, VkCommandBuffer cmd,
                     const uint8_t* frame_data, int frame_stride,
                     const SubsensePushConstants& params,
                     VkBuffer gpu_input = VK_NULL_HANDLE);

    // Engine-driven coalesced submit (Phase 7). Records the per-frame
    // process dispatches into the supplied secondary cmd buffer and returns
    // without submitting. Caller must guarantee model_ready() is true (i.e.
    // run_init has completed); first-frame init still uses the plugin's own
    // cmd_buf, same as mog2_bgs.
    bool record(VulkanContext& ctx, VkCommandBuffer cmd,
                const uint8_t* frame_data, int frame_stride,
                const uint8_t* detect_mask, int mask_stride,
                const SubsensePushConstants& params,
                VkBuffer gpu_input = VK_NULL_HANDLE);

    VkBuffer input_buffer() const { return bufs_[0]; }
    VkDeviceSize frame_byte_size() const { return frame_bytes_; }

    // GPU-resident packed GRAY8 output. Returns whichever buffer the compute
    // just wrote into: in wide layout the pack_mask shader produces
    // packed_mask_buf_; in non-wide layout (today's only active path) the
    // process shader writes packed GRAY8 directly into bufs_[4] (output),
    // and packed_mask_buf_ stays empty — returning it would register a zero
    // buffer for GPU-resident downstream consumers.
    VkBuffer packed_mask_buffer() const {
        return use_wide_layout_ ? packed_mask_buf_ : bufs_[4];
    }
    VkDeviceMemory packed_mask_memory() const {
        return use_wide_layout_ ? packed_mask_mem_ : mems_[4];
    }
    VkDeviceSize packed_mask_bytes() const { return mask_bytes_; }
    const void* staging_output_mapped() const { return staging_out_.mapped; }
    const StagingBuffer& output_staging() const { return staging_out_; }
    void invalidate_staging_output(VulkanContext& ctx) { staging_out_.invalidate(ctx); }

    void destroy(VulkanContext& ctx);

    bool initialized() const { return initialized_; }
    bool model_ready() const { return model_initialized_; }

private:
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

    // 5 buffers: input, bg_colors, bg_descs, detect_mask, output
    static constexpr int NUM_BUFFERS = 5;
    VkBuffer bufs_[NUM_BUFFERS]{};
    VkDeviceMemory mems_[NUM_BUFFERS]{};

    // packed GRAY8 output buffer (binding 5)
    VkBuffer packed_mask_buf_ = VK_NULL_HANDLE;
    VkDeviceMemory packed_mask_mem_ = VK_NULL_HANDLE;

    // push descriptor buffer cache
    VkBuffer push_bufs_[NUM_BUFFERS + 1] = {};
    VkDeviceSize push_sizes_[NUM_BUFFERS + 1] = {};

    // staging buffers (host-visible, persistently mapped)
    StagingBuffer staging_in_;
    StagingBuffer staging_out_;
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
