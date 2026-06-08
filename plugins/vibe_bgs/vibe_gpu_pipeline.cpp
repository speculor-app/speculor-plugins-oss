#ifdef SPC_HAS_VULKAN

#include "vibe_gpu_pipeline.h"

// embedded SPIR-V bytecode (generated at build time)
#include "vibe_init_comp.spv.h"
#include "vibe_process_comp.spv.h"
#include "vibe_init_wide_comp.spv.h"
#include "vibe_process_wide_comp.spv.h"
#include "vibe_pack_mask_comp.spv.h"

#include <algorithm>
#include <cstring>

namespace spc::gpu {

VibeGpuPipeline::VibeGpuPipeline() = default;

VibeGpuPipeline::~VibeGpuPipeline() = default;

bool VibeGpuPipeline::init(VulkanContext& ctx)
{
    if (initialized_) return true;
    if (!init_base(ctx)) return false;

    // descriptor set layout: 5 storage buffers (input, bg_model, output, detect_mask, packed_mask)
    desc_layout_ = ctx.has_push_descriptors
        ? spc::gpu::create_push_descriptor_layout(ctx, 5)
        : spc::gpu::create_storage_buffer_layout(ctx, 5);
    if (desc_layout_ == VK_NULL_HANDLE)
        return false;

    pipeline_layout_ = spc::gpu::create_pipeline_layout(ctx, desc_layout_, sizeof(VibePushConstants));
    if (pipeline_layout_ == VK_NULL_HANDLE)
    { destroy(ctx); return false; }

    // create all compute pipelines from embedded SPIR-V
    // packed layout (fallback)
    if (!spc::gpu::create_compute_pipeline(ctx, vibe_init_comp_spv, sizeof(vibe_init_comp_spv), pipeline_layout_, init_pipeline_))
    { destroy(ctx); return false; }
    if (!spc::gpu::create_compute_pipeline(ctx, vibe_process_comp_spv, sizeof(vibe_process_comp_spv), pipeline_layout_, process_pipeline_))
    { destroy(ctx); return false; }
    // wide layout (preferred — no atomics)
    if (!spc::gpu::create_compute_pipeline(ctx, vibe_init_wide_comp_spv, sizeof(vibe_init_wide_comp_spv), pipeline_layout_, init_wide_pipeline_))
    { destroy(ctx); return false; }
    if (!spc::gpu::create_compute_pipeline(ctx, vibe_process_wide_comp_spv, sizeof(vibe_process_wide_comp_spv), pipeline_layout_, process_wide_pipeline_))
    { destroy(ctx); return false; }
    if (!spc::gpu::create_compute_pipeline(ctx, vibe_pack_mask_comp_spv, sizeof(vibe_pack_mask_comp_spv), pipeline_layout_, pack_mask_pipeline_))
    { destroy(ctx); return false; }

    if (ctx.has_shader_objects) {
        VkPushConstantRange push_range{};
        push_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        push_range.size = sizeof(VibePushConstants);
        init_shader_ = spc::gpu::create_shader_object(ctx, vibe_init_comp_spv,
            sizeof(vibe_init_comp_spv), &desc_layout_, 1, &push_range, 1);
        process_shader_ = spc::gpu::create_shader_object(ctx, vibe_process_comp_spv,
            sizeof(vibe_process_comp_spv), &desc_layout_, 1, &push_range, 1);
        init_wide_shader_ = spc::gpu::create_shader_object(ctx, vibe_init_wide_comp_spv,
            sizeof(vibe_init_wide_comp_spv), &desc_layout_, 1, &push_range, 1);
        process_wide_shader_ = spc::gpu::create_shader_object(ctx, vibe_process_wide_comp_spv,
            sizeof(vibe_process_wide_comp_spv), &desc_layout_, 1, &push_range, 1);
        pack_mask_shader_ = spc::gpu::create_shader_object(ctx, vibe_pack_mask_comp_spv,
            sizeof(vibe_pack_mask_comp_spv), &desc_layout_, 1, &push_range, 1);
    }

    if (!ctx.has_push_descriptors)
    {
        desc_pool_ = spc::gpu::create_descriptor_pool(ctx, 5);
        if (desc_pool_ == VK_NULL_HANDLE)
        { destroy(ctx); return false; }

        desc_set_ = spc::gpu::allocate_descriptor_set(ctx, desc_pool_, desc_layout_);
        if (desc_set_ == VK_NULL_HANDLE)
        { destroy(ctx); return false; }
    }

    initialized_ = true;
    return true;
}

bool VibeGpuPipeline::prepare(VulkanContext& ctx, uint32_t width, uint32_t height,
                               int num_channels, int bytes_per_channel, int bg_samples)
{
    if (width == width_ && height == height_ &&
        num_channels == num_channels_ && bytes_per_channel == bytes_per_channel_ &&
        bg_samples == bg_samples_)
        return true;

    // wait for any in-flight work before destroying referenced resources
    wait_timeline_idle(ctx);

    // free old buffers
    spc::gpu::destroy_buffer(ctx, input_buf_, input_mem_);
    spc::gpu::destroy_buffer(ctx, bg_model_buf_, bg_model_mem_);
    spc::gpu::destroy_buffer(ctx, output_buf_, output_mem_);
    spc::gpu::destroy_buffer(ctx, detect_mask_buf_, detect_mask_mem_);
    spc::gpu::destroy_buffer(ctx, packed_mask_buf_, packed_mask_mem_);
    staging_in_.destroy(ctx);
    staging_out_.destroy(ctx);
    staging_mask_.destroy(ctx);

    width_ = width;
    height_ = height;
    num_channels_ = num_channels;
    bytes_per_channel_ = bytes_per_channel;
    bg_samples_ = bg_samples;
    model_initialized_ = false;
    pixel_count_ = width * height;

    // compute sizes
    frame_byte_size_ = static_cast<VkDeviceSize>(width) * height * num_channels * bytes_per_channel;
    mask_byte_size_ = static_cast<VkDeviceSize>(width) * height;

    VkDeviceSize model_buf_size = frame_byte_size_ * bg_samples;

    // wide output mask: one uint32 per pixel eliminates atomic contention
    // on output writes (every pixel writes, 4 neighbors share a packed word)
    // bg_model stays byte-packed (reads are the hot path, writes are sparse)
    VkDeviceSize wide_output_size = static_cast<VkDeviceSize>(pixel_count_) * sizeof(uint32_t);

    // for now, disable wide layout to establish persistent-mapping baseline
    use_wide_layout_ = false;

    if (use_wide_layout_)
    {
        output_buf_size_ = wide_output_size;
        staging_out_size_ = wide_output_size;
    }
    else
    {
        output_buf_size_ = mask_byte_size_;
        staging_out_size_ = mask_byte_size_;
    }

    // ensure minimum buffer size
    frame_byte_size_ = std::max(frame_byte_size_, VkDeviceSize(4));
    mask_byte_size_ = std::max(mask_byte_size_, VkDeviceSize(4));
    model_buf_size = std::max(model_buf_size, VkDeviceSize(4));
    output_buf_size_ = std::max(output_buf_size_, VkDeviceSize(4));
    staging_out_size_ = std::max(staging_out_size_, VkDeviceSize(4));

    // device-local buffers
    auto device_usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                        VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                        VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    auto device_mem = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;

    use_bar_input_ = false;
    if (!spc::gpu::create_buffer(ctx, frame_byte_size_, device_usage, device_mem, input_buf_, input_mem_))
    { width_ = 0; return false; }
    if (!spc::gpu::create_buffer(ctx, model_buf_size, device_usage, device_mem, bg_model_buf_, bg_model_mem_))
    { width_ = 0; return false; }
    if (!spc::gpu::create_buffer(ctx, output_buf_size_, device_usage, device_mem, output_buf_, output_mem_))
    { width_ = 0; return false; }
    if (!spc::gpu::create_buffer(ctx, mask_byte_size_, device_usage, device_mem, detect_mask_buf_, detect_mask_mem_))
    { width_ = 0; return false; }
    // packed GRAY8 output buffer (binding 4) — for GPU-resident mask output
    if (!spc::gpu::create_buffer(ctx, mask_byte_size_,
                       VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                       device_mem, packed_mask_buf_, packed_mask_mem_))
    { width_ = 0; return false; }

    // staging buffers (persistently mapped via StagingBuffer)
    if (!staging_in_.allocate(ctx, frame_byte_size_, VK_BUFFER_USAGE_TRANSFER_SRC_BIT))
    { width_ = 0; return false; }
    if (!staging_out_.allocate(ctx, staging_out_size_, VK_BUFFER_USAGE_TRANSFER_DST_BIT))
    { width_ = 0; return false; }
    if (!staging_mask_.allocate(ctx, mask_byte_size_, VK_BUFFER_USAGE_TRANSFER_SRC_BIT))
    { width_ = 0; return false; }

    // populate push descriptor cache
    push_bufs_[0] = input_buf_;     push_sizes_[0] = frame_byte_size_;
    push_bufs_[1] = bg_model_buf_;  push_sizes_[1] = model_buf_size;
    push_bufs_[2] = output_buf_;    push_sizes_[2] = output_buf_size_;
    push_bufs_[3] = detect_mask_buf_; push_sizes_[3] = mask_byte_size_;
    push_bufs_[4] = packed_mask_buf_; push_sizes_[4] = mask_byte_size_;

    if (!ctx.has_push_descriptors)
    {
        VkDescriptorBufferInfo buf_infos[5]{};
        buf_infos[0] = {input_buf_, 0, frame_byte_size_};
        buf_infos[1] = {bg_model_buf_, 0, model_buf_size};
        buf_infos[2] = {output_buf_, 0, output_buf_size_};
        buf_infos[3] = {detect_mask_buf_, 0, mask_byte_size_};
        buf_infos[4] = {packed_mask_buf_, 0, mask_byte_size_};

        VkWriteDescriptorSet writes[5]{};
        for (int i = 0; i < 5; ++i)
        {
            writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[i].dstSet = desc_set_;
            writes[i].dstBinding = static_cast<uint32_t>(i);
            writes[i].descriptorCount = 1;
            writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[i].pBufferInfo = &buf_infos[i];
        }
        vkUpdateDescriptorSets(ctx.device, 5, writes, 0, nullptr);
    }

    return true;
}

bool VibeGpuPipeline::run_init(VulkanContext& ctx,
                                const uint8_t* frame_data, uint32_t frame_size,
                                const VibePushConstants& params,
                                VkBuffer gpu_input)
{
    if (gpu_input == VK_NULL_HANDLE)
        std::memcpy(staging_in_.mapped, frame_data, frame_size);

    VkPipeline pipeline = use_wide_layout_ ? init_wide_pipeline_ : init_pipeline_;
    VkShaderEXT shader = use_wide_layout_ ? init_wide_shader_ : init_shader_;

    if (!begin_recording(ctx)) return false;
    cmd_upload_input(staging_in_, input_buf_, frame_size, gpu_input);
    barrier_transfer_to_compute();

    cmd_dispatch_compute(ctx, pipeline, shader,
                         pipeline_layout_, desc_set_,
                         push_bufs_, push_sizes_, 5,
                         params,
                         (params.width + 15) / 16, (params.height + 15) / 16);

    if (!submit_and_wait(ctx)) return false;

    model_initialized_ = true;
    return true;
}

bool VibeGpuPipeline::record_init(VulkanContext& ctx, VkCommandBuffer cmd,
                                   const uint8_t* frame_data, uint32_t frame_size,
                                   const VibePushConstants& params,
                                   VkBuffer gpu_input)
{
    if (!ctx.valid) return false;
    if (gpu_input == VK_NULL_HANDLE) {
        if (!frame_data) return false;
        std::memcpy(staging_in_.mapped, frame_data, frame_size);
    }

    ScopedExternalRecording scope(*this, cmd);

    cmd_upload_input(staging_in_, input_buf_, frame_size, gpu_input);
    barrier_transfer_to_compute();

    VkPipeline pipeline = use_wide_layout_ ? init_wide_pipeline_ : init_pipeline_;
    VkShaderEXT shader  = use_wide_layout_ ? init_wide_shader_   : init_shader_;
    cmd_dispatch_compute(ctx, pipeline, shader,
                         pipeline_layout_, desc_set_,
                         push_bufs_, push_sizes_, 5,
                         params,
                         (params.width + 15) / 16, (params.height + 15) / 16);

    model_initialized_ = true;
    return true;
}

bool VibeGpuPipeline::record(VulkanContext& ctx, VkCommandBuffer cmd,
                              const uint8_t* frame_data, uint32_t frame_size,
                              const uint8_t* detect_mask, uint32_t mask_size,
                              const VibePushConstants& params,
                              VkBuffer gpu_input)
{
    if (!ctx.valid || !frame_data || !model_initialized_) return false;

    // Stage the input frame (only if upstream isn't already GPU-resident).
    // Same CPU memcpy as run_process; the secondary cmd buffer the engine
    // handed us doesn't change anything about staging — that's plain mapped
    // memory.
    if (gpu_input == VK_NULL_HANDLE)
        std::memcpy(staging_in_.mapped, frame_data, frame_size);

    bool has_mask = (detect_mask != nullptr && mask_size > 0);
    if (has_mask)
        std::memcpy(staging_mask_.mapped, detect_mask, mask_size);

    ScopedExternalRecording scope(*this, cmd);

    // upload frame + optional mask
    cmd_upload_input(staging_in_, input_buf_, frame_size, gpu_input);
    if (has_mask)
        cmd_upload_input(staging_mask_, detect_mask_buf_, mask_size);
    barrier_transfer_to_compute();

    // encode mask presence in frame_number high bit (matches run_process)
    VibePushConstants pc = params;
    if (has_mask)
        pc.frame_number |= 0x80000000u;

    // pass 1: process
    VkPipeline pipeline = use_wide_layout_ ? process_wide_pipeline_ : process_pipeline_;
    VkShaderEXT shader  = use_wide_layout_ ? process_wide_shader_   : process_shader_;
    cmd_dispatch_compute(ctx, pipeline, shader,
                         pipeline_layout_, desc_set_,
                         push_bufs_, push_sizes_, 5,
                         pc,
                         (params.width + 15) / 16, (params.height + 15) / 16);

    // pass 2: pack mask (wide layout only)
    if (use_wide_layout_)
    {
        barrier_compute_to_compute();
        uint32_t pack_groups = ((pixel_count_ + 3) / 4 + 255) / 256;
        cmd_dispatch_compute(ctx, pack_mask_pipeline_, pack_mask_shader_,
                             pipeline_layout_, desc_set_,
                             push_bufs_, push_sizes_, 5,
                             pc, pack_groups, 1);
    }

    // readback to staging — engine handles the wait + invalidate via the registry.
    VkBuffer out_buf = use_wide_layout_ ? packed_mask_buf_ : output_buf_;
    barrier_compute_to_transfer();
    cmd_download_to_staging(out_buf, staging_out_, mask_byte_size_);
    return true;
}

void VibeGpuPipeline::destroy(VulkanContext& ctx)
{
    if (ctx.device == VK_NULL_HANDLE) return;

    ctx.wait_idle();

    spc::gpu::destroy_buffer(ctx, input_buf_, input_mem_);
    spc::gpu::destroy_buffer(ctx, bg_model_buf_, bg_model_mem_);
    spc::gpu::destroy_buffer(ctx, output_buf_, output_mem_);
    spc::gpu::destroy_buffer(ctx, detect_mask_buf_, detect_mask_mem_);
    spc::gpu::destroy_buffer(ctx, packed_mask_buf_, packed_mask_mem_);
    staging_in_.destroy(ctx);
    staging_out_.destroy(ctx);
    staging_mask_.destroy(ctx);

    spc::gpu::destroy_shader_object(ctx, init_shader_); init_shader_ = VK_NULL_HANDLE;
    spc::gpu::destroy_shader_object(ctx, process_shader_); process_shader_ = VK_NULL_HANDLE;
    spc::gpu::destroy_shader_object(ctx, init_wide_shader_); init_wide_shader_ = VK_NULL_HANDLE;
    spc::gpu::destroy_shader_object(ctx, process_wide_shader_); process_wide_shader_ = VK_NULL_HANDLE;
    spc::gpu::destroy_shader_object(ctx, pack_mask_shader_); pack_mask_shader_ = VK_NULL_HANDLE;
    if (process_wide_pipeline_ != VK_NULL_HANDLE)
        vkDestroyPipeline(ctx.device, process_wide_pipeline_, nullptr);
    if (init_wide_pipeline_ != VK_NULL_HANDLE)
        vkDestroyPipeline(ctx.device, init_wide_pipeline_, nullptr);
    if (pack_mask_pipeline_ != VK_NULL_HANDLE)
        vkDestroyPipeline(ctx.device, pack_mask_pipeline_, nullptr);
    if (process_pipeline_ != VK_NULL_HANDLE)
        vkDestroyPipeline(ctx.device, process_pipeline_, nullptr);
    if (init_pipeline_ != VK_NULL_HANDLE)
        vkDestroyPipeline(ctx.device, init_pipeline_, nullptr);
    if (desc_pool_ != VK_NULL_HANDLE)
        vkDestroyDescriptorPool(ctx.device, desc_pool_, nullptr);
    if (pipeline_layout_ != VK_NULL_HANDLE)
        vkDestroyPipelineLayout(ctx.device, pipeline_layout_, nullptr);
    if (desc_layout_ != VK_NULL_HANDLE)
        vkDestroyDescriptorSetLayout(ctx.device, desc_layout_, nullptr);

    destroy_base(ctx);
    process_wide_pipeline_ = VK_NULL_HANDLE;
    init_wide_pipeline_ = VK_NULL_HANDLE;
    process_pipeline_ = VK_NULL_HANDLE;
    init_pipeline_ = VK_NULL_HANDLE;
    desc_pool_ = VK_NULL_HANDLE;
    desc_set_ = VK_NULL_HANDLE;
    pipeline_layout_ = VK_NULL_HANDLE;
    desc_layout_ = VK_NULL_HANDLE;
    initialized_ = false;
    model_initialized_ = false;
    use_wide_layout_ = false;
}

} // namespace spc::gpu

#endif // SPC_HAS_VULKAN
