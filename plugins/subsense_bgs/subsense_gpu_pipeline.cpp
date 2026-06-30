#ifdef SPC_HAS_VULKAN

#include "subsense_gpu_pipeline.h"

// embedded SPIR-V bytecode (generated at build time)
#include "subsense_init_comp.spv.h"
#include "subsense_process_comp.spv.h"
#include "subsense_pack_mask_comp.spv.h"

#include <algorithm>
#include <cstring>

namespace spc::gpu {

SubsenseGpuPipeline::SubsenseGpuPipeline() = default;

SubsenseGpuPipeline::~SubsenseGpuPipeline() = default;

bool SubsenseGpuPipeline::init(VulkanContext& ctx)
{
    if (initialized_) return true;
    if (!ctx.valid || !ctx.device) return false;
    if (!init_base(ctx)) return false;

    // descriptor set layout: 6 storage buffers
    // (input, bg_colors, bg_descs, detect_mask, output, packed_mask)
    constexpr int TOTAL_BINDINGS = NUM_BUFFERS + 1;
    desc_layout_ = ctx.has_push_descriptors
        ? spc::gpu::create_push_descriptor_layout(ctx, TOTAL_BINDINGS)
        : spc::gpu::create_storage_buffer_layout(ctx, TOTAL_BINDINGS);
    if (desc_layout_ == VK_NULL_HANDLE)
        return false;

    pipeline_layout_ = spc::gpu::create_pipeline_layout(ctx, desc_layout_, sizeof(SubsensePushConstants));
    if (pipeline_layout_ == VK_NULL_HANDLE)
    { destroy(ctx); return false; }

    // create compute pipelines from embedded SPIR-V
    if (!spc::gpu::create_compute_pipeline(ctx, subsense_init_comp_spv,
            sizeof(subsense_init_comp_spv), pipeline_layout_, init_pipeline_))
    { destroy(ctx); return false; }

    if (!spc::gpu::create_compute_pipeline(ctx, subsense_process_comp_spv,
            sizeof(subsense_process_comp_spv), pipeline_layout_, process_pipeline_))
    { destroy(ctx); return false; }

    if (!spc::gpu::create_compute_pipeline(ctx, subsense_pack_mask_comp_spv,
            sizeof(subsense_pack_mask_comp_spv), pipeline_layout_, pack_mask_pipeline_))
    { destroy(ctx); return false; }

    if (ctx.has_shader_objects) {
        VkPushConstantRange push_range{};
        push_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        push_range.size = sizeof(SubsensePushConstants);
        init_shader_ = spc::gpu::create_shader_object(ctx, subsense_init_comp_spv,
            sizeof(subsense_init_comp_spv), &desc_layout_, 1, &push_range, 1);
        process_shader_ = spc::gpu::create_shader_object(ctx, subsense_process_comp_spv,
            sizeof(subsense_process_comp_spv), &desc_layout_, 1, &push_range, 1);
        pack_mask_shader_ = spc::gpu::create_shader_object(ctx, subsense_pack_mask_comp_spv,
            sizeof(subsense_pack_mask_comp_spv), &desc_layout_, 1, &push_range, 1);
    }

    if (!ctx.has_push_descriptors)
    {
        desc_pool_ = spc::gpu::create_descriptor_pool(ctx, TOTAL_BINDINGS);
        if (desc_pool_ == VK_NULL_HANDLE)
        { destroy(ctx); return false; }

        desc_set_ = spc::gpu::allocate_descriptor_set(ctx, desc_pool_, desc_layout_);
        if (desc_set_ == VK_NULL_HANDLE)
        { destroy(ctx); return false; }
    }

    initialized_ = true;
    return true;
}

bool SubsenseGpuPipeline::prepare(VulkanContext& ctx, uint32_t width, uint32_t height,
                                   int num_channels, int bg_samples)
{
    if (width == width_ && height == height_ &&
        num_channels == num_channels_ && bg_samples == bg_samples_)
        return true;

    // wait for any in-flight work
    wait_timeline_idle(ctx);

    // free old buffers (input upload + output are engine-owned ring slots now)
    staging_mask_.destroy(ctx);
    for (int i = 0; i < NUM_BUFFERS; ++i)
        spc::gpu::destroy_buffer(ctx, bufs_[i], mems_[i]);
    spc::gpu::destroy_buffer(ctx, packed_mask_buf_, packed_mask_mem_);

    width_ = width;
    height_ = height;
    num_channels_ = num_channels;
    bg_samples_ = bg_samples;
    model_initialized_ = false;
    mask_upload_needed_ = true;  // mask buffer realloc'd → re-upload on next record
    pixel_count_ = width * height;
    use_wide_layout_ = false;

    auto pixels = static_cast<VkDeviceSize>(width) * height;

    // compute buffer sizes
    frame_bytes_ = ((pixels * num_channels + 3) / 4) * 4;
    mask_bytes_ = ((pixels + 3) / 4) * 4;
    bg_color_bytes_ = ((static_cast<VkDeviceSize>(bg_samples) * pixels * num_channels + 3) / 4) * 4;
    // bg_descs: bg_samples * pixels uint16 values, packed as uint32
    bg_desc_bytes_ = ((static_cast<VkDeviceSize>(bg_samples) * pixels * 2 + 3) / 4) * 4;

    // ensure minimum sizes
    frame_bytes_ = std::max(frame_bytes_, VkDeviceSize(4));
    mask_bytes_ = std::max(mask_bytes_, VkDeviceSize(4));
    bg_color_bytes_ = std::max(bg_color_bytes_, VkDeviceSize(4));
    bg_desc_bytes_ = std::max(bg_desc_bytes_, VkDeviceSize(4));

    auto device_usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                        VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                        VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    auto device_mem = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;

    // buf[0] (input) is an engine upload ring slot — not allocated here.

    // buf[1]: bg_colors
    if (!spc::gpu::create_buffer(ctx, bg_color_bytes_, device_usage, device_mem, bufs_[1], mems_[1]))
    { width_ = 0; return false; }

    // buf[2]: bg_descs
    if (!spc::gpu::create_buffer(ctx, bg_desc_bytes_, device_usage, device_mem, bufs_[2], mems_[2]))
    { width_ = 0; return false; }

    // buf[3]: detect_mask
    if (!spc::gpu::create_buffer(ctx, mask_bytes_, device_usage, device_mem, bufs_[3], mems_[3]))
    { width_ = 0; return false; }

    // buf[4] (output) is an engine output ring slot — not allocated here.

    // packed GRAY8 output buffer (binding 5) — bound for descriptor validity
    if (!spc::gpu::create_buffer(ctx, mask_bytes_,
                                  VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                  device_mem, packed_mask_buf_, packed_mask_mem_))
    { width_ = 0; return false; }

    // staging for the optional detect mask (binding 3 upload source). The input
    // frame upload + output mask download staging are engine-owned ring slots.
    if (!staging_mask_.allocate(ctx, mask_bytes_, VK_BUFFER_USAGE_TRANSFER_SRC_BIT))
    { width_ = 0; return false; }

    // populate push descriptor cache. Bindings 0 (input) and 4 (output) are
    // engine ring slots re-pointed per frame; seed them null here.
    push_bufs_[0] = VK_NULL_HANDLE;  push_sizes_[0] = frame_bytes_;
    push_bufs_[1] = bufs_[1];        push_sizes_[1] = bg_color_bytes_;
    push_bufs_[2] = bufs_[2];        push_sizes_[2] = bg_desc_bytes_;
    push_bufs_[3] = bufs_[3];        push_sizes_[3] = mask_bytes_;
    push_bufs_[4] = VK_NULL_HANDLE;  push_sizes_[4] = mask_bytes_;
    push_bufs_[5] = packed_mask_buf_; push_sizes_[5] = mask_bytes_;

    // Non-push-descriptor path writes the stable bindings (1/2/3/5) once here;
    // bindings 0/4 are written per-frame in record()/record_init().
    if (!ctx.has_push_descriptors)
    {
        const uint32_t bindings[4] = {1, 2, 3, 5};
        VkDescriptorBufferInfo buf_infos[4]{};
        buf_infos[0] = {bufs_[1], 0, bg_color_bytes_};
        buf_infos[1] = {bufs_[2], 0, bg_desc_bytes_};
        buf_infos[2] = {bufs_[3], 0, mask_bytes_};
        buf_infos[3] = {packed_mask_buf_, 0, mask_bytes_};

        VkWriteDescriptorSet writes[4]{};
        for (int i = 0; i < 4; ++i)
        {
            writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[i].dstSet = desc_set_;
            writes[i].dstBinding = bindings[i];
            writes[i].descriptorCount = 1;
            writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[i].pBufferInfo = &buf_infos[i];
        }
        vkUpdateDescriptorSets(ctx.device, 4, writes, 0, nullptr);
    }

    return true;
}

// Re-point the per-frame ring bindings: 0 (input) and 4 (output). For the
// push-descriptor path this updates the cache the dispatch reads; for the
// classic descriptor-set path it writes the set. `out` may be VK_NULL_HANDLE
// for the init pass (which doesn't write the output binding).
void SubsenseGpuPipeline::set_ring_bindings(VulkanContext& ctx, VkBuffer in, VkBuffer out)
{
    push_bufs_[0] = in;
    if (out != VK_NULL_HANDLE) push_bufs_[4] = out;

    if (!ctx.has_push_descriptors)
    {
        VkDescriptorBufferInfo infos[2]{};
        VkWriteDescriptorSet writes[2]{};
        uint32_t n = 0;
        infos[n] = {in, 0, frame_bytes_};
        writes[n].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[n].dstSet = desc_set_; writes[n].dstBinding = 0;
        writes[n].descriptorCount = 1;
        writes[n].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[n].pBufferInfo = &infos[n];
        ++n;
        if (out != VK_NULL_HANDLE) {
            infos[n] = {out, 0, mask_bytes_};
            writes[n].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[n].dstSet = desc_set_; writes[n].dstBinding = 4;
            writes[n].descriptorCount = 1;
            writes[n].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[n].pBufferInfo = &infos[n];
            ++n;
        }
        vkUpdateDescriptorSets(ctx.device, n, writes, 0, nullptr);
    }
}

bool SubsenseGpuPipeline::record_init(VulkanContext& ctx, VkCommandBuffer cmd,
                                       VkBuffer in_device, VkBuffer in_staging,
                                       VkBuffer out_device,
                                       const SubsensePushConstants& params,
                                       VkBuffer gpu_input)
{
    if (!ctx.valid) return false;
    if (out_device == VK_NULL_HANDLE) return false;

    // Input binding 0 = upstream GPU output (device-to-device) or the upload
    // ring slot the plugin already filled. Binding 4 (output) gets a valid
    // buffer so the descriptor set is fully bound even though the init shader
    // doesn't write it.
    VkBuffer in_buf = (gpu_input != VK_NULL_HANDLE) ? gpu_input : in_device;
    if (in_buf == VK_NULL_HANDLE) return false;
    set_ring_bindings(ctx, in_buf, out_device);

    ScopedExternalRecording scope(*this, cmd);

    // Upload only when CPU-fed: copy the upload ring slot's staging into its
    // device buffer (the CPU memcpy into staging already happened in the
    // plugin). When gpu_input != NULL the input is read device-to-device.
    if (gpu_input == VK_NULL_HANDLE) {
        StagingBuffer in_stg{}; in_stg.buffer = in_staging;
        cmd_upload_input(in_stg, in_device, frame_bytes_);
    }
    barrier_transfer_to_compute();

    uint32_t gx = (params.width + 15) / 16;
    uint32_t gy = (params.height + 15) / 16;
    cmd_dispatch_compute(ctx, init_pipeline_, init_shader_,
                         pipeline_layout_, desc_set_,
                         push_bufs_, push_sizes_, NUM_BUFFERS + 1,
                         params, gx, gy);

    model_initialized_ = true;
    return true;
}

bool SubsenseGpuPipeline::record(VulkanContext& ctx, VkCommandBuffer cmd,
                                  VkBuffer in_device, VkBuffer in_staging,
                                  VkBuffer out_device,
                                  const uint8_t* detect_mask, int mask_stride,
                                  const SubsensePushConstants& params,
                                  VkBuffer gpu_input)
{
    if (!ctx.valid || !model_initialized_) return false;
    if (out_device == VK_NULL_HANDLE) return false;

    VkBuffer in_buf = (gpu_input != VK_NULL_HANDLE) ? gpu_input : in_device;
    if (in_buf == VK_NULL_HANDLE) return false;

    // Only refresh the persistent mask device buffer when it actually changed
    // (the shader still applies it every frame via params.has_mask).
    const bool do_upload = (detect_mask != nullptr && params.has_mask != 0) && mask_upload_needed_;
    if (do_upload) {
        auto* mask_staging = static_cast<uint8_t*>(staging_mask_.mapped);
        if (mask_stride == static_cast<int>(width_)) {
            std::memcpy(mask_staging, detect_mask, static_cast<size_t>(width_) * height_);
        } else {
            for (uint32_t y = 0; y < height_; ++y)
                std::memcpy(mask_staging + y * width_, detect_mask + y * mask_stride, width_);
        }
    }

    // Re-point bindings 0 (input ring slot) and 4 (output ring slot) at this
    // frame's buffers. At depth==1 these are the single slots, byte-identical
    // to the prior single-buffer path.
    set_ring_bindings(ctx, in_buf, out_device);

    ScopedExternalRecording scope(*this, cmd);

    // Frame upload: device-to-device (gpu_input) or staging-to-device (the
    // upload ring slot the plugin already memcpy'd into).
    if (gpu_input == VK_NULL_HANDLE) {
        StagingBuffer in_stg{}; in_stg.buffer = in_staging;
        cmd_upload_input(in_stg, in_device, frame_bytes_);
    }
    if (do_upload) {
        cmd_upload_input(staging_mask_, bufs_[3], mask_bytes_);
        mask_upload_needed_ = false;
    }
    barrier_transfer_to_compute();

    uint32_t gx = (params.width + 15) / 16;
    uint32_t gy = (params.height + 15) / 16;
    cmd_dispatch_compute(ctx, process_pipeline_, process_shader_,
                         pipeline_layout_, desc_set_,
                         push_bufs_, push_sizes_, NUM_BUFFERS + 1,
                         params, gx, gy);

    if (use_wide_layout_) {
        barrier_compute_to_compute();
        uint32_t pack_words  = (pixel_count_ + 3) / 4;
        uint32_t pack_groups = (pack_words + 255) / 256;
        cmd_dispatch_compute(ctx, pack_mask_pipeline_, pack_mask_shader_,
                             pipeline_layout_, desc_set_,
                             push_bufs_, push_sizes_, NUM_BUFFERS + 1,
                             params, pack_groups, 1);
    }
    return true;
}

void SubsenseGpuPipeline::destroy(VulkanContext& ctx)
{
    if (!ctx.device) return;

    // input upload + output buffers are engine-owned now — nothing to free here.
    staging_mask_.destroy(ctx);
    for (int i = 0; i < NUM_BUFFERS; ++i)
        spc::gpu::destroy_buffer(ctx, bufs_[i], mems_[i]);
    spc::gpu::destroy_buffer(ctx, packed_mask_buf_, packed_mask_mem_);

    spc::gpu::destroy_shader_object(ctx, init_shader_); init_shader_ = VK_NULL_HANDLE;
    spc::gpu::destroy_shader_object(ctx, process_shader_); process_shader_ = VK_NULL_HANDLE;
    spc::gpu::destroy_shader_object(ctx, pack_mask_shader_); pack_mask_shader_ = VK_NULL_HANDLE;
    if (pack_mask_pipeline_ != VK_NULL_HANDLE)
    { vkDestroyPipeline(ctx.device, pack_mask_pipeline_, nullptr); pack_mask_pipeline_ = VK_NULL_HANDLE; }
    if (process_pipeline_ != VK_NULL_HANDLE)
    { vkDestroyPipeline(ctx.device, process_pipeline_, nullptr); process_pipeline_ = VK_NULL_HANDLE; }
    if (init_pipeline_ != VK_NULL_HANDLE)
    { vkDestroyPipeline(ctx.device, init_pipeline_, nullptr); init_pipeline_ = VK_NULL_HANDLE; }
    if (desc_pool_ != VK_NULL_HANDLE)
    { vkDestroyDescriptorPool(ctx.device, desc_pool_, nullptr); desc_pool_ = VK_NULL_HANDLE; }
    if (pipeline_layout_ != VK_NULL_HANDLE)
    { vkDestroyPipelineLayout(ctx.device, pipeline_layout_, nullptr); pipeline_layout_ = VK_NULL_HANDLE; }
    if (desc_layout_ != VK_NULL_HANDLE)
    { vkDestroyDescriptorSetLayout(ctx.device, desc_layout_, nullptr); desc_layout_ = VK_NULL_HANDLE; }

    destroy_base(ctx);
    desc_set_ = VK_NULL_HANDLE;
    initialized_ = false;
    model_initialized_ = false;
    width_ = 0;
    height_ = 0;
}

} // namespace spc::gpu

#endif // SPC_HAS_VULKAN
