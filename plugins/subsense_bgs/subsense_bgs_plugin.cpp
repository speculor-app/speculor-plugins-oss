#include <speculor/plugin_helpers.h>
#include <cv_helpers.h>
#include <speculor/spclib_log_bridge.h>

#include <bgs/subsense/SuBSENSE.hpp>

#include <opencv2/core.hpp>

#include <memory>

#ifdef SPC_HAS_VULKAN
#include <gpu/vulkan_context.h>
#include <gpu/gpu_buffer_registry.h>
#include <gpu/gpu_output_handle.h>
#include <gpu/gpu_failure_tracker.h>
#include "subsense_gpu_pipeline.h"
#endif

// internal state
struct SubsenseBgsState
{
    spc::HostServices host;
    std::unique_ptr<spclib::bgs::SuBSENSE> subsense;
    cv::Mat input_image;
    cv::Mat fg_mask;
    SpcFrame output_frame;

    // cached params
    int32_t bg_samples;
    int32_t required_matches;
    float initial_color_threshold;
    int32_t initial_desc_threshold;
    float learning_rate_lower;
    float learning_rate_upper;
    // cached detection mask
    cv::Mat cached_mask;
    cv::Mat empty_mask;
    bool has_cached_mask;

#ifdef SPC_HAS_VULKAN
    // GPU state
    std::shared_ptr<spc::gpu::VulkanContext> gpu_ctx;
    std::unique_ptr<spc::gpu::SubsenseGpuPipeline> gpu_pipeline;
    bool gpu_init_attempted;
    bool gpu_available;
    uint32_t gpu_frame_counter;
    spc::gpu::GpuOutputHandle gpu_output;
    spc::gpu::GpuFailureTracker gpu_failure{"SuBSENSE"};
#endif
};

SPC_PLUGIN_CAST(SubsenseBgsState)
SPC_PLUGIN_HOST_SERVICES(SubsenseBgsState, host)

SPC_PLUGIN_DESCRIPTOR(
    spc::DescriptorBuilder("subsense_bgs", "SuBSENSE BGS", "Analysis/Motion")
        .author("Speculor").version("0.2.0")
        .description("SuBSENSE background subtraction — high-quality color+texture BGS with adaptive per-pixel sensitivity (quality mode, not real-time at full resolution)")
        .maturity(SPC_MATURITY_EXPERIMENTAL)
        .tags({"image", "tracking", "surveillance"})
        .input("image_in", "Image", SPC_DATA_FRAME, 32, SPC_CONSUME_FIFO)
        .input("mask_in", "Detect Mask", SPC_DATA_FRAME, 4, SPC_CONSUME_NON_BLOCKING)
        .output("mask_out", "FG Mask", SPC_DATA_FRAME)
        .output("image_out", "Image", SPC_DATA_FRAME)
        .gpu_compute()
        .int_param("bg_samples", "BG Samples", 10, 100, 50, 5, "SuBSENSE")
            .param_description("Number of background samples per pixel (restarts model on change)")
        .int_param("required_matches", "Required Matches", 1, 10, 2, 1, "SuBSENSE")
            .param_description("Minimum matching background samples to classify as background")
        .float_param("initial_color_threshold", "Color Threshold", 1.0f, 255.0f, 30.0f, 1.0f, "SuBSENSE")
            .param_description("Initial per-pixel color distance threshold (adapts over time on CPU, fixed on GPU)")
        .int_param("initial_desc_threshold", "Descriptor Threshold", 1, 16, 3, 1, "SuBSENSE")
            .param_description("Initial LBSP hamming distance threshold (adapts over time on CPU, fixed on GPU)")
        .float_param("learning_rate_lower", "LR Lower Bound", 0.001f, 1.0f, 0.01f, 0.001f, "SuBSENSE")
            .param_description("Minimum adaptive learning rate per pixel (CPU only)")
        .float_param("learning_rate_upper", "LR Upper Bound", 0.001f, 1.0f, 0.1f, 0.001f, "SuBSENSE")
            .param_description("Maximum adaptive learning rate per pixel (CPU only)")
        .streaming().frame_alloc()
)

// --- lifecycle ---

static SpcPluginInstance* create_instance()
{
    auto* s = new SubsenseBgsState{};
    s->bg_samples = 50;
    s->required_matches = 2;
    s->initial_color_threshold = 30.0f;
    s->initial_desc_threshold = 3;
    s->learning_rate_lower = 0.01f;
    s->learning_rate_upper = 0.1f;
    std::memset(&s->output_frame, 0, sizeof(SpcFrame));
    s->has_cached_mask = false;
    s->subsense = std::make_unique<spclib::bgs::SuBSENSE>(spclib::bgs::SuBSENSEParams());
#ifdef SPC_HAS_VULKAN
    s->gpu_init_attempted = false;
    s->gpu_available = false;
    s->gpu_frame_counter = 0;
#endif
    return reinterpret_cast<SpcPluginInstance*>(s);
}

static void destroy_instance(SpcPluginInstance* inst)
{
    auto* s = state(inst);
#ifdef SPC_HAS_VULKAN
    if (s->gpu_ctx)
        s->gpu_output.release(s->gpu_ctx.get());
    if (s->gpu_pipeline && s->gpu_ctx)
        s->gpu_pipeline->destroy(*s->gpu_ctx);
    s->gpu_pipeline.reset();
    s->gpu_ctx.reset();
#endif
    s->subsense.reset();
    delete s;
}

// --- GPU helpers ---

#ifdef SPC_HAS_VULKAN
static bool try_init_gpu(SubsenseBgsState* s)
{
    if (s->gpu_init_attempted) return s->gpu_available;
    s->gpu_init_attempted = true;

    s->gpu_ctx = spc::gpu::VulkanContext::get_shared();
    if (!s->gpu_ctx)
    {
        SPC_LOG_WARN(&s->host.cached_log, "SuBSENSE GPU: no Vulkan device found, falling back to CPU");
        return false;
    }

    s->gpu_pipeline = std::make_unique<spc::gpu::SubsenseGpuPipeline>();
    if (!s->gpu_pipeline->init(*s->gpu_ctx))
    {
        SPC_LOG_WARN(&s->host.cached_log, "SuBSENSE GPU: pipeline init failed, falling back to CPU");
        s->gpu_pipeline->destroy(*s->gpu_ctx);
        s->gpu_pipeline.reset();
        s->gpu_ctx.reset();
        return false;
    }

    SPC_LOG_INFO(&s->host.cached_log, "SuBSENSE GPU: initialized on %s",
                 s->gpu_ctx->device_name().c_str());
    s->gpu_available = true;
    return true;
}

#endif

// --- parameters ---

static int set_parameter(SpcPluginInstance* inst, const char* name, const SpcParameterDesc* value)
{
    auto* s = state(inst);
    if (!s->subsense) return -1;

    if (spc::try_set_int(name, value, "bg_samples", s->bg_samples))
    {
        s->subsense->get_parameters().set_bg_samples(s->bg_samples);
#ifdef SPC_HAS_VULKAN
        // force GPU model re-init on sample count change
        if (s->gpu_pipeline)
        {
            // prepare() will detect the parameter change and re-allocate
        }
#endif
    }
    else if (spc::try_set_int(name, value, "required_matches", s->required_matches))
    {
        s->subsense->get_parameters().set_required_matches(s->required_matches);
    }
    else if (spc::try_set_float(name, value, "initial_color_threshold", s->initial_color_threshold))
    {
        s->subsense->get_parameters().set_initial_color_threshold(s->initial_color_threshold);
    }
    else if (spc::try_set_int(name, value, "initial_desc_threshold", s->initial_desc_threshold))
    {
        s->subsense->get_parameters().set_initial_desc_threshold(s->initial_desc_threshold);
    }
    else if (spc::try_set_float(name, value, "learning_rate_lower", s->learning_rate_lower))
    {
        s->subsense->get_parameters().set_learning_rate_lower(s->learning_rate_lower);
    }
    else if (spc::try_set_float(name, value, "learning_rate_upper", s->learning_rate_upper))
    {
        s->subsense->get_parameters().set_learning_rate_upper(s->learning_rate_upper);
    }
    else
    {
        return -1;
    }
    return 0;
}

static int get_parameter(SpcPluginInstance* inst, const char* name, SpcParameterDesc* out)
{
    auto* s = state(inst);
    if (spc::try_get_int(name, out, "bg_samples", s->bg_samples)) return 0;
    if (spc::try_get_int(name, out, "required_matches", s->required_matches)) return 0;
    if (spc::try_get_float(name, out, "initial_color_threshold", s->initial_color_threshold)) return 0;
    if (spc::try_get_int(name, out, "initial_desc_threshold", s->initial_desc_threshold)) return 0;
    if (spc::try_get_float(name, out, "learning_rate_lower", s->learning_rate_lower)) return 0;
    if (spc::try_get_float(name, out, "learning_rate_upper", s->learning_rate_upper)) return 0;
    return -1;
}

// --- streaming ---

static int start(SpcPluginInstance* inst)
{
    auto* s = state(inst);
    spc::install_spclib_log_bridge(&s->host.cached_log);
    if (s->subsense)
    {
        s->subsense->restart();
    }
    s->cached_mask = cv::Mat();
    s->has_cached_mask = false;
#ifdef SPC_HAS_VULKAN
    s->gpu_init_attempted = false;
    s->gpu_available = false;
    s->gpu_frame_counter = 0;
    // Eager GPU init — record_gpu (engine coalesced path) bypasses process()
    // entirely, so the lazy try_init_gpu inside process() never runs in
    // coalesced mode. Same shape as mog2_bgs/vibe_bgs::start.
    try_init_gpu(s);
#endif
    SPC_LOG_INFO(&s->host.cached_log, "SuBSENSE BGS started");
    return 0;
}

static int stop(SpcPluginInstance* inst)
{
    auto* s = state(inst);
#ifdef SPC_HAS_VULKAN
    if (s->gpu_ctx) {
        s->gpu_output.release(s->gpu_ctx.get());
        if (s->gpu_pipeline)
            s->gpu_pipeline->destroy(*s->gpu_ctx);
    }
    s->gpu_pipeline.reset();
    s->gpu_ctx.reset();
    s->gpu_init_attempted = false;
    s->gpu_available = false;
#endif
    SPC_LOG_INFO(&s->host.cached_log, "SuBSENSE BGS stopped");
    return 0;
}

// --- process ---

static int process(SpcPluginInstance* inst, const SpcData* inputs, uint32_t input_count,
                   SpcData* outputs, uint32_t output_count)
{
    auto* s = state(inst);

    thread_local bool bridge_installed = false;
    if (!bridge_installed) {
        spc::install_spclib_log_bridge(&s->host.cached_log);
        bridge_installed = true;
    }

    if (input_count < 1 || output_count < 2) return -1;
    if (inputs[0].type != SPC_DATA_FRAME || !inputs[0].frame) return -1;

    const SpcFrame* in_frame = inputs[0].frame;
    int cv_type = spc::cv_type_for_format(in_frame->format);
    if (cv_type < 0) return -1;

    // SuBSENSE only supports 8-bit input
    int depth = CV_MAT_DEPTH(cv_type);
    if (depth != CV_8U) return -1;

    if (input_count > 1 && inputs[1].type == SPC_DATA_FRAME && inputs[1].frame)
    {
        const SpcFrame* mask_frame = inputs[1].frame;
        cv::Mat temp_mask(static_cast<int>(mask_frame->height),
                         static_cast<int>(mask_frame->width),
                         CV_8UC1, mask_frame->data,
                         static_cast<size_t>(mask_frame->stride));
        temp_mask.copyTo(s->cached_mask);
        s->has_cached_mask = true;
    }

    // GPU path lives entirely in record_gpu (Phase 8). process() is the
    // CPU fallback — entered when the engine demoted GPU for this node
    // or built the node without a subgraph executor.

    // --- CPU path ---

    s->input_image = cv::Mat(static_cast<int>(in_frame->height),
                            static_cast<int>(in_frame->width),
                            cv_type, in_frame->data,
                            static_cast<size_t>(in_frame->stride));

    auto w = static_cast<uint32_t>(in_frame->width);
    auto h = static_cast<uint32_t>(in_frame->height);

    SpcFrame* out = s->host.acquire_frame(0, w, h, SPC_PIXEL_FORMAT_GRAY8);
    if (out)
    {
        s->fg_mask = cv::Mat(static_cast<int>(h), static_cast<int>(w), CV_8UC1, out->data);
        s->subsense->apply(s->input_image, s->fg_mask,
                           s->has_cached_mask ? s->cached_mask : s->empty_mask);
        out->frame_number = in_frame->frame_number;
        out->timestamp_us = in_frame->timestamp_us;
        outputs[0].type = SPC_DATA_FRAME;
        outputs[0].frame = out;
        outputs[1].type = SPC_DATA_FRAME;
        outputs[1].frame = const_cast<SpcFrame*>(in_frame);
        return 0;
    }

    // fallback: no allocator
    s->subsense->apply(s->input_image, s->fg_mask,
                       s->has_cached_mask ? s->cached_mask : s->empty_mask);
    const cv::Mat& fg_mat = s->fg_mask;
    spc::mat_to_frame(fg_mat, &s->output_frame, SPC_PIXEL_FORMAT_GRAY8,
                     in_frame->frame_number, in_frame->timestamp_us);
    outputs[0].type = SPC_DATA_FRAME;
    outputs[0].frame = &s->output_frame;
    outputs[1].type = SPC_DATA_FRAME;
    outputs[1].frame = const_cast<SpcFrame*>(in_frame);
    return 0;
}

// --- record_gpu (engine-driven coalesced submit, Phase 7) ---
//
// Mirrors process_gpu but records dispatches into the engine's secondary
// command buffer instead of running its own submit_and_wait. First-frame
// model init also records into the engine secondary (see record_init);
// a plugin-private submit would race the engine's not-yet-submitted
// coalesced secondary and read uninitialized memory from upstream.

#ifdef SPC_HAS_VULKAN
static int record_gpu(SpcPluginInstance* inst, SpcGpuRecordCtx* rctx)
{
    auto* s = state(inst);
    if (!rctx || rctx->struct_size < sizeof(SpcGpuRecordCtx)) return -1;
    if (rctx->input_count < 1 || rctx->output_count < 2) return -1;
    if (rctx->inputs[0].type != SPC_DATA_FRAME || !rctx->inputs[0].frame) return -1;
    if (!s->gpu_available || !s->gpu_ctx || !s->gpu_pipeline) return -1;

    const SpcFrame* in_frame = rctx->inputs[0].frame;
    int cv_type = spc::cv_type_for_format(in_frame->format);
    if (cv_type < 0) return -1;

    if (rctx->input_count > 1 && rctx->inputs[1].type == SPC_DATA_FRAME && rctx->inputs[1].frame) {
        const SpcFrame* mask_frame = rctx->inputs[1].frame;
        if (mask_frame->format == SPC_PIXEL_FORMAT_GRAY8) {
            cv::Mat temp_mask(static_cast<int>(mask_frame->height),
                              static_cast<int>(mask_frame->width),
                              CV_8UC1, mask_frame->data,
                              static_cast<size_t>(mask_frame->stride));
            temp_mask.copyTo(s->cached_mask);
            s->has_cached_mask = true;
        }
    }

    auto w = static_cast<uint32_t>(in_frame->width);
    auto h = static_cast<uint32_t>(in_frame->height);

    int num_channels = 1;
    switch (in_frame->format) {
    case SPC_PIXEL_FORMAT_RGB24:
    case SPC_PIXEL_FORMAT_BGR24: num_channels = 3; break;
    case SPC_PIXEL_FORMAT_GRAY8:
    default:                     num_channels = 1; break;
    }

    if (!s->gpu_pipeline->prepare(*s->gpu_ctx, w, h, num_channels, s->bg_samples))
        return -1;

    bool input_on_gpu = in_frame->gpu_handle != 0 &&
                        (in_frame->gpu_flags & SPC_GPU_FLAG_RESIDENT);
    VkBuffer gpu_input_buf = VK_NULL_HANDLE;
    if (input_on_gpu) {
        auto entry = spc::gpu::GpuBufferRegistry::instance().lookup(in_frame->gpu_handle);
        if (entry.buffer != VK_NULL_HANDLE) gpu_input_buf = entry.buffer;
    }

    spc::gpu::SubsensePushConstants pc{};
    pc.width = static_cast<int32_t>(w);
    pc.height = static_cast<int32_t>(h);
    pc.num_channels = num_channels;
    pc.bg_samples = s->bg_samples;
    pc.color_threshold = static_cast<int32_t>(s->initial_color_threshold);
    pc.desc_threshold = s->initial_desc_threshold;
    pc.required_matches = s->required_matches;
    pc.learning_rate = 16;
    pc.frame_number = s->gpu_frame_counter++;
    pc.has_mask = s->has_cached_mask ? 1 : 0;
    pc.width4 = (static_cast<int32_t>(w) + 3) / 4;

    // First-frame model init: record into the engine secondary so the read
    // of gpu_input is properly ordered after upstream writes. A plugin-
    // private submit_and_wait here would race the engine's not-yet-
    // submitted coalesced secondary and seed the background model from
    // uninitialized GPU memory.
    if (!s->gpu_pipeline->model_ready()) {
        auto init_cmd = static_cast<VkCommandBuffer>(rctx->cmd_buffer_handle);
        if (!init_cmd) return -1;
        if (!s->gpu_pipeline->record_init(*s->gpu_ctx, init_cmd,
                                          in_frame->data,
                                          static_cast<int>(in_frame->stride),
                                          pc, gpu_input_buf))
            return -1;
        SpcFrame* out = s->host.acquire_frame(0, w, h, SPC_PIXEL_FORMAT_GRAY8);
        if (out) {
            std::memset(out->data, 0, static_cast<size_t>(out->stride) * h);
            out->frame_number = in_frame->frame_number;
            out->timestamp_us = in_frame->timestamp_us;
            rctx->outputs[0].type = SPC_DATA_FRAME;
            rctx->outputs[0].frame = out;
        }
        rctx->outputs[1].type = SPC_DATA_FRAME;
        rctx->outputs[1].frame = const_cast<SpcFrame*>(in_frame);
        return 0;
    }

    SpcFrame* out = s->host.acquire_frame(0, w, h, SPC_PIXEL_FORMAT_GRAY8);
    if (!out) return -1;

    auto cmd = static_cast<VkCommandBuffer>(rctx->cmd_buffer_handle);
    if (!cmd) {
        s->host.release_frame(out);
        return -1;
    }

    const uint8_t* mask_ptr = nullptr;
    int mask_stride = 0;
    if (s->has_cached_mask && !s->cached_mask.empty()) {
        mask_ptr = s->cached_mask.data;
        mask_stride = static_cast<int>(s->cached_mask.step[0]);
    }

    if (!s->gpu_pipeline->record(*s->gpu_ctx, cmd,
                                  in_frame->data, static_cast<int>(in_frame->stride),
                                  mask_ptr, mask_stride,
                                  pc, gpu_input_buf)) {
        s->host.release_frame(out);
        return -1;
    }

    s->gpu_output.bind_to_frame(s->gpu_ctx, out, in_frame,
                                 s->gpu_pipeline->packed_mask_buffer(),
                                 s->gpu_pipeline->packed_mask_memory(),
                                 s->gpu_pipeline->packed_mask_bytes(),
                                 s->gpu_pipeline->staging_output_mapped(),
                                 &s->gpu_pipeline->output_staging());
    out->frame_number = in_frame->frame_number;
    out->timestamp_us = in_frame->timestamp_us;
    rctx->outputs[0].type = SPC_DATA_FRAME;
    rctx->outputs[0].frame = out;
    rctx->outputs[1].type = SPC_DATA_FRAME;
    rctx->outputs[1].frame = const_cast<SpcFrame*>(in_frame);
    return 0;
}
#endif

#ifdef SPC_HAS_VULKAN
SPC_PLUGIN_VTABLE(
    .get_descriptor    = get_descriptor,
    .create_instance   = create_instance,
    .destroy_instance  = destroy_instance,
    .set_parameter     = set_parameter,
    .get_parameter     = get_parameter,
    .process           = process,
    .start             = start,
    .stop              = stop,
    .set_host_services = set_host_services,
    .record_gpu        = record_gpu
)
#else
SPC_PLUGIN_VTABLE(
    .get_descriptor    = get_descriptor,
    .create_instance   = create_instance,
    .destroy_instance  = destroy_instance,
    .set_parameter     = set_parameter,
    .get_parameter     = get_parameter,
    .process           = process,
    .start             = start,
    .stop              = stop,
    .set_host_services = set_host_services
)
#endif
