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

    // free old buffers (input upload + output are engine-owned now)
    spc::gpu::destroy_buffer(ctx, bg_model_buf_, bg_model_mem_);
    spc::gpu::destroy_buffer(ctx, detect_mask_buf_, detect_mask_mem_);
    spc::gpu::destroy_buffer(ctx, packed_mask_buf_, packed_mask_mem_);
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

    // device-local buffers (plugin-owned: bg_model + detect_mask + packed).
    // The input upload buffer (binding 0) and output buffer (binding 2) are
    // engine-owned ring slots, resolved per-frame in record()/record_init().
    auto device_usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                        VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                        VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    auto device_mem = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;

    if (!spc::gpu::create_buffer(ctx, model_buf_size, device_usage, device_mem, bg_model_buf_, bg_model_mem_))
    { width_ = 0; return false; }
    if (!spc::gpu::create_buffer(ctx, mask_byte_size_, device_usage, device_mem, detect_mask_buf_, detect_mask_mem_))
    { width_ = 0; return false; }
    // packed GRAY8 output buffer (binding 4) — for GPU-resident mask output
    if (!spc::gpu::create_buffer(ctx, mask_byte_size_,
                       VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                       device_mem, packed_mask_buf_, packed_mask_mem_))
    { width_ = 0; return false; }

    // staging buffer for the optional detect_mask (binding 3 upload source).
    if (!staging_mask_.allocate(ctx, mask_byte_size_, VK_BUFFER_USAGE_TRANSFER_SRC_BIT))
    { width_ = 0; return false; }

    // populate push descriptor cache. Bindings 0 (input) and 2 (output) are
    // engine ring slots re-pointed per frame; seed them null here.
    push_bufs_[0] = VK_NULL_HANDLE; push_sizes_[0] = frame_byte_size_;
    push_bufs_[1] = bg_model_buf_;  push_sizes_[1] = model_buf_size;
    push_bufs_[2] = VK_NULL_HANDLE; push_sizes_[2] = output_buf_size_;
    push_bufs_[3] = detect_mask_buf_; push_sizes_[3] = mask_byte_size_;
    push_bufs_[4] = packed_mask_buf_; push_sizes_[4] = mask_byte_size_;

    // Non-push-descriptor path writes the stable bindings (1/3/4) once here;
    // bindings 0/2 are written per-frame in record()/record_init().
    if (!ctx.has_push_descriptors)
    {
        VkDescriptorBufferInfo buf_infos[3]{};
        buf_infos[0] = {bg_model_buf_, 0, model_buf_size};
        buf_infos[1] = {detect_mask_buf_, 0, mask_byte_size_};
        buf_infos[2] = {packed_mask_buf_, 0, mask_byte_size_};
        const uint32_t bindings[3] = {1, 3, 4};

        VkWriteDescriptorSet writes[3]{};
        for (int i = 0; i < 3; ++i)
        {
            writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[i].dstSet = desc_set_;
            writes[i].dstBinding = bindings[i];
            writes[i].descriptorCount = 1;
            writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[i].pBufferInfo = &buf_infos[i];
        }
        vkUpdateDescriptorSets(ctx.device, 3, writes, 0, nullptr);
    }

    return true;
}

// Re-point the per-frame ring bindings: 0 (input) and 2 (output). For the
// push-descriptor path this just updates the cache the dispatch reads; for the
// classic descriptor-set path it writes the set. `out` may be VK_NULL_HANDLE
// for the init pass (which doesn't write the output binding) — only binding 0
// is set in that case.
void VibeGpuPipeline::set_ring_bindings(VulkanContext& ctx, VkBuffer in, VkBuffer out)
{
    push_bufs_[0] = in;
    if (out != VK_NULL_HANDLE) push_bufs_[2] = out;

    if (!ctx.has_push_descriptors)
    {
        VkDescriptorBufferInfo infos[2]{};
        VkWriteDescriptorSet writes[2]{};
        uint32_t n = 0;
        infos[n] = {in, 0, frame_byte_size_};
        writes[n].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[n].dstSet = desc_set_; writes[n].dstBinding = 0;
        writes[n].descriptorCount = 1;
        writes[n].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[n].pBufferInfo = &infos[n];
        ++n;
        if (out != VK_NULL_HANDLE) {
            infos[n] = {out, 0, output_buf_size_};
            writes[n].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[n].dstSet = desc_set_; writes[n].dstBinding = 2;
            writes[n].descriptorCount = 1;
            writes[n].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[n].pBufferInfo = &infos[n];
            ++n;
        }
        vkUpdateDescriptorSets(ctx.device, n, writes, 0, nullptr);
    }
}

bool VibeGpuPipeline::record_init(VulkanContext& ctx, VkCommandBuffer cmd,
                                   VkBuffer in_device, VkBuffer in_staging,
                                   uint32_t frame_size, VkBuffer out_device,
                                   const VibePushConstants& params,
                                   VkBuffer gpu_input)
{
    if (!ctx.valid) return false;
    if (out_device == VK_NULL_HANDLE) return false;
    // Input binding 0 = upstream GPU output (device-to-device) or the upload
    // ring slot the plugin already filled. The init dispatch reads binding 0.
    // Binding 2 (output) gets a valid buffer too so the descriptor set is fully
    // bound even though the init shader doesn't write it.
    VkBuffer in_buf = (gpu_input != VK_NULL_HANDLE) ? gpu_input : in_device;
    if (in_buf == VK_NULL_HANDLE) return false;
    set_ring_bindings(ctx, in_buf, out_device);

    ScopedExternalRecording scope(*this, cmd);

    // Upload only when CPU-fed (gpu_input == NULL): copy the upload ring
    // slot's staging into its device buffer. The CPU memcpy into staging
    // already happened in the plugin. When gpu_input != NULL the input is read
    // directly from the upstream output (binding 0 set above) — no copy.
    if (gpu_input == VK_NULL_HANDLE) {
        StagingBuffer in_stg{}; in_stg.buffer = in_staging;
        cmd_upload_input(in_stg, in_device, frame_size);
        barrier_transfer_to_compute();
    }

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
                              VkBuffer in_device, VkBuffer in_staging, uint32_t frame_size,
                              VkBuffer out_device, VkBuffer out_staging,
                              const uint8_t* detect_mask, uint32_t mask_size,
                              const VibePushConstants& params,
                              VkBuffer gpu_input)
{
    if (!ctx.valid || !model_initialized_) return false;
    if (out_device == VK_NULL_HANDLE || out_staging == VK_NULL_HANDLE) return false;

    // Input binding 0 = upstream GPU output (device-to-device) or the upload
    // ring slot the plugin already memcpy'd the CPU frame into.
    VkBuffer in_buf = (gpu_input != VK_NULL_HANDLE) ? gpu_input : in_device;
    if (in_buf == VK_NULL_HANDLE) return false;

    bool has_mask = (detect_mask != nullptr && mask_size > 0);
    if (has_mask)
        std::memcpy(staging_mask_.mapped, detect_mask, mask_size);

    // Re-point bindings 0 (input ring slot) and 2 (output ring slot) at this
    // frame's buffers. At depth==1 these are the single slots, so this rewires
    // the same buffers every frame (the engine's ensure_registered caches the
    // output handle) — byte-identical to the prior single-buffer path. At
    // depth>1 it routes this frame into slots distinct from in-flight frames'.
    set_ring_bindings(ctx, in_buf, out_device);

    ScopedExternalRecording scope(*this, cmd);

    // upload frame (only when CPU-fed; gpu_input is read directly) + optional
    // mask. Barrier fires when either upload was recorded.
    bool recorded_transfer = false;
    if (gpu_input == VK_NULL_HANDLE) {
        StagingBuffer in_stg{}; in_stg.buffer = in_staging;
        cmd_upload_input(in_stg, in_device, frame_size);
        recorded_transfer = true;
    }
    if (has_mask) {
        cmd_upload_input(staging_mask_, detect_mask_buf_, mask_size);
        recorded_transfer = true;
    }
    if (recorded_transfer)
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

    // readback to the OUTPUT ring slot's staging — engine handles the wait +
    // invalidate via the registry. Non-wide (active) path downloads the
    // process shader's direct output (out_device); wide layout downloads
    // packed_mask_buf_ into the same staging.
    VkBuffer out_buf = use_wide_layout_ ? packed_mask_buf_ : out_device;
    StagingBuffer out_stg{}; out_stg.buffer = out_staging;
    barrier_compute_to_transfer();
    cmd_download_to_staging(out_buf, out_stg, mask_byte_size_);
    return true;
}

void VibeGpuPipeline::destroy(VulkanContext& ctx)
{
    if (ctx.device == VK_NULL_HANDLE) return;

    ctx.wait_idle();

    // input upload + output buffers are engine-owned now — nothing to free here.
    spc::gpu::destroy_buffer(ctx, bg_model_buf_, bg_model_mem_);
    spc::gpu::destroy_buffer(ctx, detect_mask_buf_, detect_mask_mem_);
    spc::gpu::destroy_buffer(ctx, packed_mask_buf_, packed_mask_mem_);
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
