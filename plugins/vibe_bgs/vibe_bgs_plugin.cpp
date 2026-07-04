#include <speculor/plugin_helpers.h>
#include <cv_helpers.h>
#include <speculor/spclib_log_bridge.h>

#include <bgs/vibe/Vibe.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <atomic>
#include <memory>

#ifdef SPC_HAS_VULKAN
#include <gpu/vulkan_context.h>
#include <gpu/gpu_buffer_registry.h>
#include <gpu/gpu_output_handle.h>
#include <gpu/gpu_failure_tracker.h>
#include "vibe_gpu_pipeline.h"
#endif

// GUI-thread-set parameters, snapshotted on the worker (H6). The ViBe detector
// is worker-owned — its setters must run on the worker from a snapshot, not
// from set_parameter, so the CPU process() applies the snapshot on a dirty flag
// and record_gpu reads the snapshot directly.
// Defaults retuned 2026-07 end-to-end (ViBe→dual_morph→blob_detect→sort_tracker
// against drone-sim GT on real all-sky footage, tracker at v0.4.0): the earlier
// threshold 35 / learning_rate 8 point was compensating for the pre-fix tracker,
// which turned extra detections into ID churn. With matching fixed, higher
// sensitivity + slower background absorption wins on recall, precision, false
// tracks, and ID stability simultaneously.
struct Params
{
    int32_t threshold = 27;
    int32_t bg_samples = 16;
    int32_t required_bg_samples = 1;
    int32_t learning_rate = 16;
};

// internal state
struct VibeBgsState
{
    spc::HostServices host;
    std::unique_ptr<spclib::bgs::Vibe> vibe;
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
    bool mask_warned;      // logged the non-GRAY8 mask perf warning once
    bool mask_dim_warned;  // logged the mask/frame dimension-mismatch warning once

#ifdef SPC_HAS_VULKAN
    // GPU state
    std::shared_ptr<spc::gpu::VulkanContext> gpu_ctx;
    std::unique_ptr<spc::gpu::VibeGpuPipeline> gpu_pipeline;
    bool gpu_init_attempted;
    bool gpu_available;
    bool gpu_model_initialized;
    uint32_t gpu_frame_counter;
    spc::gpu::GpuOutputHandle gpu_output;
    spc::gpu::GpuFailureTracker gpu_failure{"ViBe"};
#endif
};

SPC_PLUGIN_CAST(VibeBgsState)
SPC_PLUGIN_HOST_SERVICES(VibeBgsState, host)


SPC_PLUGIN_DESCRIPTOR(
    spc::DescriptorBuilder("vibe_bgs", "ViBe BGS", "Analysis/Motion")
        .author("Speculor").version("0.3.0")
        .description("ViBe background subtraction — outputs foreground mask")
        .maturity(SPC_MATURITY_STABLE)
        .tags({"image", "tracking", "surveillance"})
        .input("image_in", "Image", SPC_DATA_FRAME, 32, SPC_CONSUME_FIFO)
        .input("mask_in", "Detect Mask", SPC_DATA_FRAME, 4, SPC_CONSUME_NON_BLOCKING)
        .output("mask_out", "FG Mask", SPC_DATA_FRAME)
        .output("image_out", "Image", SPC_DATA_FRAME)
        .gpu_compute()
        .int_param("threshold", "Threshold", 1, 255, 27, 1, "ViBe")
            .param_description("Pixel intensity difference to classify as foreground")
        .int_param("bg_samples", "BG Samples", 2, 64, 16, 2, "ViBe")
            .param_description("Number of background samples stored per pixel")
        .int_param("required_bg_samples", "Required BG Samples", 1, 16, 1, 1, "ViBe")
            .param_description("How many samples must match for a pixel to be classified as background")
        .int_param("learning_rate", "Learning Rate", 1, 32, 16, 1, "ViBe")
            .param_description("Chance (1 in N) that a foreground pixel updates the background model — "
                               "higher = slower absorption, keeps slow/hovering movers foreground longer")
        .streaming().frame_alloc()
)

// --- lifecycle ---

// apply a parameter snapshot to the worker-owned ViBe detector (worker thread)
static void apply_params(VibeBgsState* s, const Params& p)
{
    if (!s->vibe) return;
    auto& vp = s->vibe->get_parameters();
    vp.set_threshold(static_cast<uint32_t>(p.threshold));
    vp.set_bg_samples(static_cast<uint32_t>(p.bg_samples));
    vp.set_required_bg_samples(static_cast<uint32_t>(p.required_bg_samples));
    vp.set_learning_rate(static_cast<uint32_t>(p.learning_rate));
}

static SpcPluginInstance* create_instance()
{
    auto* s = new VibeBgsState{};
    // Params defaults match the descriptor; the engine validates via
    // get_parameter and pushes descriptor values on mismatch.
    std::memset(&s->output_frame, 0, sizeof(SpcFrame));
    s->has_cached_mask = false;
    s->mask_warned = false;
    s->mask_dim_warned = false;
    s->vibe = std::make_unique<spclib::bgs::Vibe>(spclib::bgs::VibeParams(), false);
#ifdef SPC_HAS_VULKAN
    s->gpu_init_attempted = false;
    s->gpu_available = false;
    s->gpu_model_initialized = false;
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
    s->vibe.reset();
    delete s;
}

// --- GPU helpers ---

#ifdef SPC_HAS_VULKAN
static bool try_init_gpu(VibeBgsState* s)
{
    if (s->gpu_init_attempted) return s->gpu_available;
    s->gpu_init_attempted = true;

    s->gpu_ctx = spc::gpu::VulkanContext::get_shared();
    if (!s->gpu_ctx)
    {
        SPC_LOG_WARN(&s->host.cached_log, "ViBe GPU: no Vulkan device found, falling back to CPU");
        return false;
    }

    s->gpu_pipeline = std::make_unique<spc::gpu::VibeGpuPipeline>();
    if (!s->gpu_pipeline->init(*s->gpu_ctx))
    {
        SPC_LOG_WARN(&s->host.cached_log, "ViBe GPU: pipeline init failed, falling back to CPU");
        s->gpu_pipeline->destroy(*s->gpu_ctx);
        s->gpu_pipeline.reset();
        s->gpu_ctx.reset();
        return false;
    }

    SPC_LOG_INFO(&s->host.cached_log, "ViBe GPU: initialized on %s (push_desc=%s, transfer_q=%s)",
                 s->gpu_ctx->device_name().c_str(),
                 s->gpu_ctx->has_push_descriptors ? "on" : "off",
                 s->gpu_ctx->has_dedicated_transfer ? "dedicated" : "shared");
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
        return spc::try_set_int(name, value, "threshold", p.threshold)
            || spc::try_set_int(name, value, "bg_samples", p.bg_samples)
            || spc::try_set_int(name, value, "required_bg_samples", p.required_bg_samples)
            || spc::try_set_int(name, value, "learning_rate", p.learning_rate);
    });
    if (matched) s->params_dirty.store(true, std::memory_order_release);
    return matched ? 0 : -1;
}

static int get_parameter(SpcPluginInstance* inst, const char* name, SpcParameterDesc* out)
{
    auto* s = state(inst);
    const Params p = s->params.snapshot();
    if (spc::try_get_int(name, out, "threshold", p.threshold)) return 0;
    if (spc::try_get_int(name, out, "bg_samples", p.bg_samples)) return 0;
    if (spc::try_get_int(name, out, "required_bg_samples", p.required_bg_samples)) return 0;
    if (spc::try_get_int(name, out, "learning_rate", p.learning_rate)) return 0;
    return -1;
}

// --- streaming ---

static int start(SpcPluginInstance* inst)
{
    auto* s = state(inst);
    spc::install_spclib_log_bridge(&s->host.cached_log);
    if (s->vibe)
    {
        s->vibe->restart();
        apply_params(s, s->params.snapshot());  // sync detector to current params
        s->params_dirty.store(false, std::memory_order_release);
    }
    s->cached_mask = cv::Mat();
    s->has_cached_mask = false;
    s->mask_warned = false;
    s->mask_dim_warned = false;
#ifdef SPC_HAS_VULKAN
    s->gpu_model_initialized = false;
    s->gpu_frame_counter = 0;
    // Eager GPU init — same pattern as mog2_bgs::start. Required for
    // record_gpu (the engine's coalesced submit path bypasses process(),
    // so the lazy init that lived in process() never runs in that mode).
    try_init_gpu(s);
#endif
    SPC_LOG_INFO(&s->host.cached_log, "ViBe BGS started");
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
    s->gpu_model_initialized = false;
#endif
    SPC_LOG_INFO(&s->host.cached_log, "ViBe BGS stopped");
    return 0;
}

// --- detection mask ---

// Cache the port-1 detection mask as single-channel GRAY8 — what both the CPU
// detector and the GPU shader expect (1 byte/pixel). GRAY8 is copied straight
// through. A color mask is converted with a full-frame cvtColor EVERY frame,
// which is costly on the hot path; warn once and prefer a GRAY8 mask source.
// Any other format is ignored (also warned once). Worker-thread only — process()
// and record_gpu() never run concurrently, so mask_warned needs no atomicity.
// Returns true when s->cached_mask was (re)written, so record_gpu knows to flag
// the GPU mask buffer for re-upload (the mask is otherwise uploaded once).
static bool cache_detect_mask(VibeBgsState* s, const SpcFrame* mask_frame)
{
    if (!mask_frame->data || mask_frame->width == 0 || mask_frame->height == 0)
        return false;

    if (mask_frame->format == SPC_PIXEL_FORMAT_GRAY8) {
        spc::frame_to_mat(mask_frame, CV_8UC1).copyTo(s->cached_mask);
        s->has_cached_mask = true;
        return true;
    }

    int code = 0;
    switch (mask_frame->format) {
        case SPC_PIXEL_FORMAT_RGB24:  code = cv::COLOR_RGB2GRAY;  break;
        case SPC_PIXEL_FORMAT_BGR24:  code = cv::COLOR_BGR2GRAY;  break;
        case SPC_PIXEL_FORMAT_RGBA32: code = cv::COLOR_RGBA2GRAY; break;
        default:
            if (!s->mask_warned) {
                SPC_LOG_WARN(&s->host.cached_log,
                    "ViBe detect mask format %d is unsupported (need GRAY8 or 8-bit "
                    "color) — ignoring the mask input.",
                    static_cast<int>(mask_frame->format));
                s->mask_warned = true;
            }
            return false;
    }

    if (!s->mask_warned) {
        SPC_LOG_WARN(&s->host.cached_log,
            "ViBe detect mask is color, not GRAY8 — converting to grayscale every "
            "frame (full-frame cvtColor, costly on the hot path). Feed a "
            "single-channel GRAY8 mask (e.g. enable Grayscale on the mask source) "
            "for best performance.");
        s->mask_warned = true;
    }

    cv::cvtColor(spc::frame_to_mat(mask_frame, spc::cv_type_for_format(mask_frame->format)),
                 s->cached_mask, code);
    s->has_cached_mask = true;
    return true;
}

// The cached mask is only usable when it matches the frame dimensions: both
// the CPU detector and the GPU staging upload index it with image-sized
// offsets, so a mismatched mask (e.g. sized for a different camera) would be
// read out of bounds. Checked per frame — the image size can change
// mid-stream. Warn once and ignore the mask.
static bool mask_usable(VibeBgsState* s, uint32_t w, uint32_t h)
{
    if (!s->has_cached_mask) return false;
    if (s->cached_mask.cols == static_cast<int>(w) &&
        s->cached_mask.rows == static_cast<int>(h))
        return true;
    if (!s->mask_dim_warned) {
        SPC_LOG_WARN(&s->host.cached_log,
            "ViBe detect mask is %dx%d but frames are %ux%u — ignoring the "
            "mask (size it to the camera resolution).",
            s->cached_mask.cols, s->cached_mask.rows, w, h);
        s->mask_dim_warned = true;
    }
    return false;
}

// --- process ---

static int process(SpcPluginInstance* inst, const SpcData* inputs, uint32_t input_count,
                   SpcData* outputs, uint32_t output_count)
{
    auto* s = state(inst);

    // ensure spclib log bridge is installed on this (node) thread
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

    if (input_count > 1 && inputs[1].type == SPC_DATA_FRAME && inputs[1].frame)
        cache_detect_mask(s, inputs[1].frame);

    // GPU path lives entirely in record_gpu (Phase 8). process() is the
    // CPU fallback — entered when the engine demoted GPU for this node
    // or built the node without a subgraph executor.

    // --- CPU path ---

    // wrap input frame data directly (zero-copy)
    s->input_image = cv::Mat(static_cast<int>(in_frame->height),
                             static_cast<int>(in_frame->width),
                             cv_type, in_frame->data,
                             static_cast<size_t>(in_frame->stride));

    // acquire pool frame and have ViBe write directly into it
    auto w = static_cast<uint32_t>(in_frame->width);
    auto h = static_cast<uint32_t>(in_frame->height);
    const bool use_mask = mask_usable(s, w, h);

    SpcFrame* out = s->host.acquire_frame(0, w, h, SPC_PIXEL_FORMAT_GRAY8);
    if (out)
    {
        // wrap pool buffer as Image — ViBe writes directly, no copy
        s->fg_mask = cv::Mat(static_cast<int>(h), static_cast<int>(w), CV_8UC1, out->data);
        s->vibe->apply(s->input_image, s->fg_mask,
                       use_mask ? s->cached_mask : s->empty_mask);
        out->frame_number = in_frame->frame_number;
        out->timestamp_ns = in_frame->timestamp_ns;
        outputs[0].type = SPC_DATA_FRAME;
        outputs[0].frame = out;
        outputs[1].type = SPC_DATA_FRAME;
        outputs[1].frame = const_cast<SpcFrame*>(in_frame);
        return 0;
    }

    // fallback: no allocator — use internal buffer
    s->vibe->apply(s->input_image, s->fg_mask,
                   use_mask ? s->cached_mask : s->empty_mask);
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
// Mirrors the GPU happy path of process() but records dispatches into the
// engine's secondary command buffer instead of running its own
// submit_and_wait. The engine concatenates this secondary with downstream
// subgraph members' secondaries (e.g. dual_morph reading vibe's mask
// output) into one primary, submits once per frame.
//
// First-frame init records the upload + init compute into the engine's
// secondary (same cmd buffer as the per-frame dispatch). This is required
// for correctness when the upstream input is GPU-resident: a plugin-
// private submit_and_wait would race the engine's not-yet-submitted
// coalesced secondary and read uninitialized memory from the upstream
// output buffer, seeding the background model with black. The engine's
// inter-member barrier in the primary makes the secondary read of
// gpu_input correctly observe the upstream compute writes.

#ifdef SPC_HAS_VULKAN
// Acquire the engine-owned K-deep INPUT upload ring slot for this frame and
// memcpy the CPU frame into its staging. Only used when the upstream input is
// NOT already GPU-resident (gpu_input == NULL). Returns the slot's device +
// staging buffers via out params; returns false if the host has no edge-ring
// service or allocation failed (caller demotes to CPU). The upload ring slot
// rotates with the executor's shared per-frame counter, so frame N's upload
// can't clobber a slot a prior in-flight submit is still reading — this is the
// last write hazard that K=2 closes on the vibe→dual_morph chain.
static bool acquire_input_upload(VibeBgsState* s, SpcGpuRecordCtx* rctx,
                                 const SpcFrame* in_frame, uint32_t frame_size,
                                 VkBuffer& in_device, VkBuffer& in_staging)
{
    in_device = VK_NULL_HANDLE;
    in_staging = VK_NULL_HANDLE;
    if (!rctx->edge_ring_ctx) return false;
    SpcGpuEdgeBuffer up = s->host.acquire_ringed_upload(
        rctx->edge_ring_ctx, 0,
        static_cast<uint64_t>(s->gpu_pipeline->input_device_size()),
        static_cast<uint64_t>(frame_size));
    if (!up.device_buffer || !up.staging_buffer || !up.staging_mapped) return false;
    std::memcpy(up.staging_mapped, in_frame->data, frame_size);
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

    // Update cached detection mask from input port 1 (same as process()); flag
    // the GPU buffer for re-upload only when the mask actually changed, so the
    // pipeline doesn't re-push an identical mask every frame.
    if (rctx->input_count > 1 && rctx->inputs[1].type == SPC_DATA_FRAME && rctx->inputs[1].frame) {
        if (cache_detect_mask(s, rctx->inputs[1].frame))
            s->gpu_pipeline->mark_mask_dirty();
    }

    auto w = static_cast<uint32_t>(in_frame->width);
    auto h = static_cast<uint32_t>(in_frame->height);

    int num_channels = 1;
    int bytes_per_channel = 1;
    switch (in_frame->format) {
    case SPC_PIXEL_FORMAT_RGB24:
    case SPC_PIXEL_FORMAT_BGR24:
        num_channels = 3; bytes_per_channel = 1; break;
    case SPC_PIXEL_FORMAT_GRAY16:
        num_channels = 1; bytes_per_channel = 2; break;
    case SPC_PIXEL_FORMAT_RGB48:
        num_channels = 3; bytes_per_channel = 2; break;
    case SPC_PIXEL_FORMAT_GRAY8:
    default:
        num_channels = 1; bytes_per_channel = 1; break;
    }

    if (!s->gpu_pipeline->prepare(*s->gpu_ctx, w, h, num_channels, bytes_per_channel, p.bg_samples))
        return -1;

    uint32_t frame_size = w * h * static_cast<uint32_t>(num_channels) *
                          static_cast<uint32_t>(bytes_per_channel);
    uint32_t mask_size  = w * h;

    bool input_on_gpu = in_frame->gpu_handle != 0 &&
                        (in_frame->gpu_flags & SPC_GPU_FLAG_RESIDENT);
    VkBuffer gpu_input_buf = VK_NULL_HANDLE;
    if (input_on_gpu) {
        auto entry = spc::gpu::GpuBufferRegistry::instance().lookup(in_frame->gpu_handle);
        if (entry.buffer != VK_NULL_HANDLE) gpu_input_buf = entry.buffer;
    }

    spc::gpu::VibePushConstants pc{};
    pc.width = static_cast<int32_t>(w);
    pc.height = static_cast<int32_t>(h);
    pc.num_channels = num_channels;
    pc.bg_samples = p.bg_samples;
    pc.threshold = (bytes_per_channel == 2) ? p.threshold * 256 : p.threshold;
    pc.required_bg_samples = p.required_bg_samples;
    auto power_of_2 = [](uint32_t v) -> uint32_t {
        v |= v >> 1; v |= v >> 2; v |= v >> 4; v |= v >> 8; v |= v >> 16;
        return v - (v >> 1);
    };
    uint32_t bg_p2 = power_of_2(static_cast<uint32_t>(p.bg_samples));
    uint32_t lr_p2 = power_of_2(static_cast<uint32_t>(p.learning_rate));
    pc.bg_samples_and = static_cast<int32_t>(bg_p2 > 1 ? bg_p2 - 1 : 1);
    pc.learning_rate_and = static_cast<int32_t>(lr_p2 > 1 ? lr_p2 - 1 : 0);
    pc.bg_samples = static_cast<int32_t>(bg_p2 > 1 ? bg_p2 : 2);
    pc.frame_number = s->gpu_frame_counter++;
    pc.bytes_per_channel = bytes_per_channel;

    auto cmd = static_cast<VkCommandBuffer>(rctx->cmd_buffer_handle);
    if (!cmd) return -1;

    SpcFrame* out = s->host.acquire_frame(0, w, h, SPC_PIXEL_FORMAT_GRAY8);
    if (!out) return -1;

    // Acquire the engine-owned OUTPUT ring slot for this frame (binding 2 +
    // download dst). The engine registers the slot's buffer + staging against
    // the returned gpu_handle so the post-submit invalidate covers it and
    // dual_morph resolves it by handle — exactly the prior single-handle
    // contract at depth==1. width/height/stride/format describe `out`. A
    // zeroed return (host predates edge rings / alloc failed) demotes to CPU.
    // Acquired for the init frame too so the init dispatch's binding 2 is a
    // valid buffer (the init shader doesn't write it, but it must be bound).
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
        !acquire_input_upload(s, rctx, in_frame, frame_size, in_device, in_staging)) {
        s->host.release_frame(out);
        return -1;
    }

    // First-frame init: record the upload + init dispatch into the engine
    // secondary so it executes after upstream writes (see comment above). The
    // first frame's mask is conventionally all-zero — emit a CPU zero mask
    // (NOT GPU-resident) + passthrough; the output ring slot was acquired only
    // to keep binding 2 valid.
    if (!s->gpu_pipeline->model_initialized()) {
        if (!s->gpu_pipeline->record_init(*s->gpu_ctx, cmd,
                                          in_device, in_staging, frame_size,
                                          static_cast<VkBuffer>(outbuf.device_buffer),
                                          pc, gpu_input_buf)) {
            s->host.release_frame(out);
            return -1;
        }
        s->gpu_model_initialized = true;
        std::memset(out->data, 0, static_cast<size_t>(out->stride) * h);
        out->frame_number = in_frame->frame_number;
        out->timestamp_ns = in_frame->timestamp_ns;
        rctx->outputs[0].type = SPC_DATA_FRAME;
        rctx->outputs[0].frame = out;
        rctx->outputs[1].type = SPC_DATA_FRAME;
        rctx->outputs[1].frame = const_cast<SpcFrame*>(in_frame);
        return 0;
    }

    const bool use_mask = mask_usable(s, w, h);
    const uint8_t* det_mask = use_mask ? s->cached_mask.data : nullptr;
    uint32_t det_mask_size  = use_mask ? mask_size : 0;

    if (!s->gpu_pipeline->record(*s->gpu_ctx, cmd,
                                  in_device, in_staging, frame_size,
                                  static_cast<VkBuffer>(outbuf.device_buffer),
                                  det_mask, det_mask_size,
                                  pc, gpu_input_buf)) {
        s->host.release_frame(out);
        return -1;
    }

    // Stamp the output frame as GPU-resident from the engine ring slot's
    // handle. The engine already registered the slot buffer + staging, so we
    // just publish the handle + frame metadata (same fields
    // GpuOutputHandle::bind_to_frame set on the prior path).
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
