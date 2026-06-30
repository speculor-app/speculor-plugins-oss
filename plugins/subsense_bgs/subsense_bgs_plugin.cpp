#include <speculor/plugin_helpers.h>
#include <cv_helpers.h>
#include <speculor/spclib_log_bridge.h>

#include <bgs/subsense/SuBSENSE.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <atomic>
#include <memory>

#ifdef SPC_HAS_VULKAN
#include <gpu/vulkan_context.h>
#include <gpu/gpu_buffer_registry.h>
#include <gpu/gpu_output_handle.h>
#include <gpu/gpu_failure_tracker.h>
#include "subsense_gpu_pipeline.h"
#endif

// GUI-thread-set parameters, snapshotted on the worker (H6). The SuBSENSE
// detector is worker-owned: CPU process() applies the snapshot on a dirty
// flag; record_gpu reads the snapshot directly.
struct Params
{
    int32_t bg_samples = 50;
    int32_t required_matches = 2;
    float initial_color_threshold = 30.0f;
    int32_t initial_desc_threshold = 3;
    float learning_rate_lower = 0.01f;
    float learning_rate_upper = 0.1f;
};

// internal state
struct SubsenseBgsState
{
    spc::HostServices host;
    std::unique_ptr<spclib::bgs::SuBSENSE> subsense;
    cv::Mat input_image;
    cv::Mat fg_mask;
    SpcFrame output_frame;

    // cross-thread parameter block (GUI writes, worker snapshots per frame)
    spc::SharedParams<Params> params;
    std::atomic<bool> params_dirty{false};
    // cached detection mask
    cv::Mat cached_mask;
    cv::Mat empty_mask;
    bool has_cached_mask;
    bool mask_warned;  // logged the non-GRAY8 mask perf warning once

#ifdef SPC_HAS_VULKAN
    // GPU state
    std::shared_ptr<spc::gpu::VulkanContext> gpu_ctx;
    std::unique_ptr<spc::gpu::SubsenseGpuPipeline> gpu_pipeline;
    bool gpu_init_attempted;
    bool gpu_available;
    uint32_t gpu_frame_counter;
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

// apply a parameter snapshot to the worker-owned SuBSENSE detector (worker thread)
static void apply_params(SubsenseBgsState* s, const Params& p)
{
    if (!s->subsense) return;
    auto& sp = s->subsense->get_parameters();
    sp.set_bg_samples(p.bg_samples);
    sp.set_required_matches(p.required_matches);
    sp.set_initial_color_threshold(p.initial_color_threshold);
    sp.set_initial_desc_threshold(p.initial_desc_threshold);
    sp.set_learning_rate_lower(p.learning_rate_lower);
    sp.set_learning_rate_upper(p.learning_rate_upper);
}

static SpcPluginInstance* create_instance()
{
    auto* s = new SubsenseBgsState{};
    std::memset(&s->output_frame, 0, sizeof(SpcFrame));
    s->has_cached_mask = false;
    s->mask_warned = false;
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
    // Mutate the shared block only; the worker applies it to the detector on
    // the dirty flag (CPU) / reads the snapshot directly (GPU).
    bool matched = s->params.update([&](Params& p) {
        return spc::try_set_int(name, value, "bg_samples", p.bg_samples)
            || spc::try_set_int(name, value, "required_matches", p.required_matches)
            || spc::try_set_float(name, value, "initial_color_threshold", p.initial_color_threshold)
            || spc::try_set_int(name, value, "initial_desc_threshold", p.initial_desc_threshold)
            || spc::try_set_float(name, value, "learning_rate_lower", p.learning_rate_lower)
            || spc::try_set_float(name, value, "learning_rate_upper", p.learning_rate_upper);
    });
    if (matched) s->params_dirty.store(true, std::memory_order_release);
    return matched ? 0 : -1;
}

static int get_parameter(SpcPluginInstance* inst, const char* name, SpcParameterDesc* out)
{
    auto* s = state(inst);
    const Params p = s->params.snapshot();
    if (spc::try_get_int(name, out, "bg_samples", p.bg_samples)) return 0;
    if (spc::try_get_int(name, out, "required_matches", p.required_matches)) return 0;
    if (spc::try_get_float(name, out, "initial_color_threshold", p.initial_color_threshold)) return 0;
    if (spc::try_get_int(name, out, "initial_desc_threshold", p.initial_desc_threshold)) return 0;
    if (spc::try_get_float(name, out, "learning_rate_lower", p.learning_rate_lower)) return 0;
    if (spc::try_get_float(name, out, "learning_rate_upper", p.learning_rate_upper)) return 0;
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
        apply_params(s, s->params.snapshot());  // sync detector to current params
        s->params_dirty.store(false, std::memory_order_release);
    }
    s->cached_mask = cv::Mat();
    s->has_cached_mask = false;
    s->mask_warned = false;
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
    if (s->gpu_ctx && s->gpu_pipeline)
        s->gpu_pipeline->destroy(*s->gpu_ctx);
    s->gpu_pipeline.reset();
    s->gpu_ctx.reset();
    s->gpu_init_attempted = false;
    s->gpu_available = false;
#endif
    SPC_LOG_INFO(&s->host.cached_log, "SuBSENSE BGS stopped");
    return 0;
}

// --- detection mask ---

// Cache the port-1 detection mask as single-channel GRAY8 — what both the CPU
// detector and the GPU shader expect (1 byte/pixel). GRAY8 is copied straight
// through. A color mask is converted with a full-frame cvtColor EVERY frame,
// which is costly on the hot path; warn once and prefer a GRAY8 mask source.
// Any other format is ignored (also warned once). Worker-thread only — process()
// and record_gpu() never run concurrently, so mask_warned needs no atomicity.
static void cache_detect_mask(SubsenseBgsState* s, const SpcFrame* mask_frame)
{
    if (!mask_frame->data || mask_frame->width == 0 || mask_frame->height == 0)
        return;

    if (mask_frame->format == SPC_PIXEL_FORMAT_GRAY8) {
        spc::frame_to_mat(mask_frame, CV_8UC1).copyTo(s->cached_mask);
        s->has_cached_mask = true;
        return;
    }

    int code = 0;
    switch (mask_frame->format) {
        case SPC_PIXEL_FORMAT_RGB24:  code = cv::COLOR_RGB2GRAY;  break;
        case SPC_PIXEL_FORMAT_BGR24:  code = cv::COLOR_BGR2GRAY;  break;
        case SPC_PIXEL_FORMAT_RGBA32: code = cv::COLOR_RGBA2GRAY; break;
        default:
            if (!s->mask_warned) {
                SPC_LOG_WARN(&s->host.cached_log,
                    "SuBSENSE detect mask format %d is unsupported (need GRAY8 or "
                    "8-bit color) — ignoring the mask input.",
                    static_cast<int>(mask_frame->format));
                s->mask_warned = true;
            }
            return;
    }

    if (!s->mask_warned) {
        SPC_LOG_WARN(&s->host.cached_log,
            "SuBSENSE detect mask is color, not GRAY8 — converting to grayscale "
            "every frame (full-frame cvtColor, costly on the hot path). Feed a "
            "single-channel GRAY8 mask (e.g. enable Grayscale on the mask source) "
            "for best performance.");
        s->mask_warned = true;
    }

    cv::cvtColor(spc::frame_to_mat(mask_frame, spc::cv_type_for_format(mask_frame->format)),
                 s->cached_mask, code);
    s->has_cached_mask = true;
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

    // apply any GUI-thread parameter change to the worker-owned detector
    if (s->params_dirty.exchange(false, std::memory_order_acquire))
        apply_params(s, s->params.snapshot());

    const SpcFrame* in_frame = inputs[0].frame;
    int cv_type = spc::cv_type_for_format(in_frame->format);
    if (cv_type < 0) return -1;

    // SuBSENSE only supports 8-bit input
    int depth = CV_MAT_DEPTH(cv_type);
    if (depth != CV_8U) return -1;

    if (input_count > 1 && inputs[1].type == SPC_DATA_FRAME && inputs[1].frame)
        cache_detect_mask(s, inputs[1].frame);

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
        out->timestamp_ns = in_frame->timestamp_ns;
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
                     in_frame->frame_number, in_frame->timestamp_ns);
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
// Acquire the engine-owned K-deep INPUT upload ring slot for this frame and
// memcpy the CPU frame into its staging. Only used when the upstream input is
// NOT already GPU-resident (gpu_input == NULL). SuBSENSE is 8-bit only (1
// byte/channel). Returns false if the host has no edge-ring service or the
// allocation failed (caller demotes to CPU).
static bool acquire_input_upload(SubsenseBgsState* s, SpcGpuRecordCtx* rctx,
                                 const SpcFrame* in_frame, int num_channels,
                                 VkBuffer& in_device, VkBuffer& in_staging)
{
    in_device = VK_NULL_HANDLE;
    in_staging = VK_NULL_HANDLE;
    if (!rctx->edge_ring_ctx) return false;
    const uint32_t w = static_cast<uint32_t>(in_frame->width);
    const uint32_t h = static_cast<uint32_t>(in_frame->height);
    const int row_bytes = static_cast<int>(w) * num_channels;
    // device + staging are sized input_device_size() (the padded frame_bytes_
    // the pipeline's cmd_upload_input copies); only the real row_bytes*h bytes
    // are memcpy'd in. Sizing staging at the unpadded frame size would overrun
    // on the padded device copy.
    const uint64_t ring_bytes = static_cast<uint64_t>(s->gpu_pipeline->input_device_size());
    SpcGpuEdgeBuffer up = s->host.acquire_ringed_upload(
        rctx->edge_ring_ctx, 0, ring_bytes, ring_bytes);
    if (!up.device_buffer || !up.staging_buffer || !up.staging_mapped) return false;
    auto* dst = static_cast<uint8_t*>(up.staging_mapped);
    const auto* src = in_frame->data;
    const int stride = static_cast<int>(in_frame->stride);
    if (stride == row_bytes)
        std::memcpy(dst, src, static_cast<size_t>(row_bytes) * h);
    else
        for (uint32_t y = 0; y < h; ++y)
            std::memcpy(dst + y * row_bytes, src + y * stride, row_bytes);
    in_device  = static_cast<VkBuffer>(up.device_buffer);
    in_staging = static_cast<VkBuffer>(up.staging_buffer);
    return true;
}

static int record_gpu(SpcPluginInstance* inst, SpcGpuRecordCtx* rctx)
{
    auto* s = state(inst);
    if (!rctx || rctx->struct_size < sizeof(SpcGpuRecordCtx)) return -1;
    if (rctx->input_count < 1 || rctx->output_count < 2) return -1;
    if (rctx->inputs[0].type != SPC_DATA_FRAME || !rctx->inputs[0].frame) return -1;
    if (!s->gpu_available || !s->gpu_ctx || !s->gpu_pipeline) return -1;

    const Params p = s->params.snapshot();  // one consistent view per frame

    const SpcFrame* in_frame = rctx->inputs[0].frame;
    int cv_type = spc::cv_type_for_format(in_frame->format);
    if (cv_type < 0) return -1;

    if (rctx->input_count > 1 && rctx->inputs[1].type == SPC_DATA_FRAME && rctx->inputs[1].frame)
        cache_detect_mask(s, rctx->inputs[1].frame);

    auto w = static_cast<uint32_t>(in_frame->width);
    auto h = static_cast<uint32_t>(in_frame->height);

    int num_channels = 1;
    switch (in_frame->format) {
    case SPC_PIXEL_FORMAT_RGB24:
    case SPC_PIXEL_FORMAT_BGR24: num_channels = 3; break;
    case SPC_PIXEL_FORMAT_GRAY8:
    default:                     num_channels = 1; break;
    }

    if (!s->gpu_pipeline->prepare(*s->gpu_ctx, w, h, num_channels, p.bg_samples))
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
    pc.bg_samples = p.bg_samples;
    pc.color_threshold = static_cast<int32_t>(p.initial_color_threshold);
    pc.desc_threshold = p.initial_desc_threshold;
    pc.required_matches = p.required_matches;
    pc.learning_rate = 16;
    pc.frame_number = s->gpu_frame_counter++;
    pc.has_mask = s->has_cached_mask ? 1 : 0;
    pc.width4 = (static_cast<int32_t>(w) + 3) / 4;

    auto cmd = static_cast<VkCommandBuffer>(rctx->cmd_buffer_handle);
    if (!cmd) return -1;

    SpcFrame* out = s->host.acquire_frame(0, w, h, SPC_PIXEL_FORMAT_GRAY8);
    if (!out) return -1;

    // Acquire the engine-owned OUTPUT ring slot for this frame (binding 4 +
    // download dst). The engine registers the slot's buffer + staging against
    // the returned gpu_handle, so the post-submit invalidate covers it and
    // downstream consumers resolve it by handle. Acquired for the init frame
    // too so the init dispatch's binding 4 is a valid buffer.
    if (!rctx->edge_ring_ctx) { s->host.release_frame(out); return -1; }
    SpcGpuEdgeBuffer outbuf = s->host.acquire_ringed_output(
        rctx->edge_ring_ctx, 0, w, h, out->stride, SPC_PIXEL_FORMAT_GRAY8,
        static_cast<uint64_t>(s->gpu_pipeline->output_device_size()),
        static_cast<uint64_t>(s->gpu_pipeline->output_staging_size()));
    // staging may be null — only the device buffer + registry handle are required
    if (!outbuf.device_buffer || outbuf.gpu_handle == 0) {
        s->host.release_frame(out);
        return -1;
    }

    // CPU-fed input → acquire the upload ring slot + memcpy the frame in.
    // GPU-resident input → device-to-device, no upload ring.
    VkBuffer in_device = VK_NULL_HANDLE, in_staging = VK_NULL_HANDLE;
    if (gpu_input_buf == VK_NULL_HANDLE &&
        !acquire_input_upload(s, rctx, in_frame, num_channels, in_device, in_staging)) {
        s->host.release_frame(out);
        return -1;
    }

    // First-frame model init: record into the engine secondary so the read of
    // gpu_input is properly ordered after upstream writes. First frame's mask
    // is conventionally all-zero — emit a CPU zero mask (NOT GPU-resident) +
    // passthrough; the output ring slot was acquired only to keep binding 4
    // valid.
    if (!s->gpu_pipeline->model_ready()) {
        if (!s->gpu_pipeline->record_init(*s->gpu_ctx, cmd,
                                          in_device, in_staging,
                                          static_cast<VkBuffer>(outbuf.device_buffer),
                                          pc, gpu_input_buf)) {
            s->host.release_frame(out);
            return -1;
        }
        std::memset(out->data, 0, static_cast<size_t>(out->stride) * h);
        out->frame_number = in_frame->frame_number;
        out->timestamp_ns = in_frame->timestamp_ns;
        rctx->outputs[0].type = SPC_DATA_FRAME;
        rctx->outputs[0].frame = out;
        rctx->outputs[1].type = SPC_DATA_FRAME;
        rctx->outputs[1].frame = const_cast<SpcFrame*>(in_frame);
        return 0;
    }

    const uint8_t* mask_ptr = nullptr;
    int mask_stride = 0;
    if (s->has_cached_mask && !s->cached_mask.empty()) {
        mask_ptr = s->cached_mask.data;
        mask_stride = static_cast<int>(s->cached_mask.step[0]);
    }

    if (!s->gpu_pipeline->record(*s->gpu_ctx, cmd,
                                  in_device, in_staging,
                                  static_cast<VkBuffer>(outbuf.device_buffer),
                                  mask_ptr, mask_stride,
                                  pc, gpu_input_buf)) {
        s->host.release_frame(out);
        return -1;
    }

    // Stamp the output frame as GPU-resident from the engine ring slot's
    // handle (same fields GpuOutputHandle::bind_to_frame set on the prior path).
    out->gpu_handle   = outbuf.gpu_handle;
    out->gpu_flags   |= SPC_GPU_FLAG_RESIDENT;
    out->frame_number = in_frame->frame_number;
    out->timestamp_ns = in_frame->timestamp_ns;
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
