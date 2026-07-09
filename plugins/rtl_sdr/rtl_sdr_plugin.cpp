#include "rtl_sdr_device.h"
#include <speculor/sdr_source_helpers.h>
#include <speculor/sdr_params.h>
#include <spc_clock.h>

#include <chrono>
#include <cstring>
#include <memory>
#include <thread>

// ── device registry ─────────────────────────────────────────────────

static struct RtlSdrRegistry {
    spc::sdr::DeviceEntry devices[spc::sdr::MAX_DEVICES];
    uint32_t device_indices[spc::sdr::MAX_DEVICES]; // map enum index to hw index
    int count = 0;

    // gain table populated per-device after open
    int gains[64];
    int gain_count = 0;

    bool initialized = false;

    void scan()
    {
        count = 0;

        // first entry = "None"
        devices[count].index = -1;
        std::strncpy(devices[count].label, "None", SPC_PARAM_ENUM_LABEL_MAX);
        device_indices[count] = UINT32_MAX;
        count++;

        if (!spc::rtlsdr::RtlSdrDevice::load_api()) {
            initialized = true;
            return;
        }

        auto hw = spc::rtlsdr::RtlSdrDevice::enumerate();
        for (const auto& dev : hw) {
            if (count >= spc::sdr::MAX_DEVICES) break;
            devices[count].index = static_cast<int>(dev.index);
            std::snprintf(devices[count].label, SPC_PARAM_ENUM_LABEL_MAX,
                          "%s [%s]", dev.name, dev.serial);
            device_indices[count] = dev.index;
            count++;
        }

        initialized = true;
    }
} g_registry;

// ── state ───────────────────────────────────────────────────────────

struct RtlSdrState {
    spc::HostServices host;

    // device
    std::unique_ptr<spc::rtlsdr::RtlSdrDevice> device;

    // parameters — Device group
    int32_t device_idx = 0;

    // parameters — Tuning group
    int32_t center_freq = 100000000; // 100 MHz
    int32_t sample_rate = 2048000; // Hz
    int32_t bandwidth = 0; // 0 = auto

    // parameters — Gain group
    int32_t agc_enabled = 1;
    float   gain_db = 30.0f; // manual tuner gain in dB, snapped to a hardware step

    // parameters — Hardware group
    int32_t direct_sampling = 0; // 0=off, 1=I-ADC, 2=Q-ADC
    int32_t offset_tuning = 0;
    int32_t bias_tee = 0;
    int32_t freq_correction = 0; // PPM
    int32_t test_mode = 0;
    int32_t dithering = 1;       // frequency dithering (improves R828D phase noise)
    int32_t if_gain_stage = 0;   // enum: stage 1-6
    int32_t if_gain = 0;         // tenths of dB

    // cached
    double actual_sample_rate = 2048000.0;

    // output
    SpcTable output_table{};
    std::vector<int16_t> batch_buffer;
    uint64_t sample_count = 0;
    bool streaming = false;
};

SPC_PLUGIN_CAST(RtlSdrState)
SPC_PLUGIN_HOST_SERVICES(RtlSdrState, host)

// ── descriptor & device scan ────────────────────────────────────────

static SpcPluginDescriptor g_desc;
static bool g_desc_initialized = false;

static void patch_device_enum()
{
    for (uint32_t i = 0; i < g_desc.param_count; ++i) {
        if (std::strcmp(g_desc.params[i].name, "device") == 0) {
            auto& ev = g_desc.params[i].enum_val;
            ev.count = g_registry.count;
            for (int j = 0; j < g_registry.count; ++j) {
                std::strncpy(ev.labels[j], g_registry.devices[j].label,
                             SPC_PARAM_ENUM_LABEL_MAX - 1);
                ev.labels[j][SPC_PARAM_ENUM_LABEL_MAX - 1] = '\0';
            }
        }
    }
}

// Snap a requested dB gain to the nearest hardware-supported step and apply it.
// The tuner only supports a discrete set of gains (queried into g_registry.gains
// at open); we keep the user-facing value a device-independent dB float so it
// persists cleanly, and snap here when applying.
static void apply_manual_gain(spc::rtlsdr::RtlSdrDevice* dev, float gain_db, SpcLogContext* log)
{
    int target = static_cast<int>(gain_db * 10.0f + (gain_db >= 0.0f ? 0.5f : -0.5f)); // tenths dB
    if (g_registry.gain_count > 0) {
        int best = g_registry.gains[0];
        int best_d = target - best; if (best_d < 0) best_d = -best_d;
        for (int i = 1; i < g_registry.gain_count; ++i) {
            int d = target - g_registry.gains[i]; if (d < 0) d = -d;
            if (d < best_d) { best_d = d; best = g_registry.gains[i]; }
        }
        target = best;
    }
    dev->set_tuner_gain(target);
}

static const SpcPluginDescriptor* scan_devices(const SpcHostServices* svc)
{
    SpcLogContext log{};
    if (svc && svc->log) log = {svc->log, svc->host_ctx};

    SPC_LOG_INFO(&log, "RTL-SDR: scanning for devices...");
    g_registry.scan();
    patch_device_enum();

    if (!spc::rtlsdr::RtlSdrDevice::is_api_loaded()) {
        SPC_LOG_WARN(&log, "RTL-SDR: rtlsdr library not found");
    } else {
        SPC_LOG_INFO(&log, "RTL-SDR: found %d device(s)", g_registry.count - 1);
        for (int i = 1; i < g_registry.count; ++i)
            SPC_LOG_INFO(&log, "RTL-SDR:   [%d] %s", i, g_registry.devices[i].label);
    }

    return &g_desc;
}

static const SpcPluginDescriptor* get_descriptor()
{
    if (!g_desc_initialized) {
        spc::DescriptorBuilder("rtl_sdr", "RTL-SDR", "Signal/SDR/Sources")
            .author("Speculor").version("0.1.0")
            .data_source()
            .description("Streams I/Q data from RTL-SDR Blog V3 (R820T2) and V4 (R828D) receivers")
            .maturity(SPC_MATURITY_PREVIEW)
            .tags({"radio"})
            .output_signal("iq_out", "I/Q Output", {
                {"i", SPC_FIELD_INT16},
                {"q", SPC_FIELD_INT16},
            })
            // Device
            .enum_param("device", "Device", {"(no devices found)"}, 0, SPC_SDR_GROUP_DEVICE)
                .param_description("Connected RTL-SDR receiver to use")
                .mandatory()
            // Tuning
            .int_param(SPC_SDR_CENTER_FREQ, "Center Freq (Hz)", 500000, 1766000000, 100000000, 1000, SPC_SDR_GROUP_TUNING)
                .param_description("Receiver center frequency in Hz (24 MHz - 1.766 GHz)")
            .int_param(SPC_SDR_SAMPLE_RATE, "Sample Rate (Hz)", 225001, 3200000, 2048000, 1000, SPC_SDR_GROUP_TUNING)
                .param_description("ADC sample rate in Hz (higher = wider bandwidth, more CPU)")
            .int_param(SPC_SDR_BANDWIDTH, "IF Bandwidth (Hz)", 0, 3200000, 0, 10000, SPC_SDR_GROUP_TUNING)
                .param_description("IF bandwidth in Hz (0 = automatic, matched to sample rate)")
            // Gain
            .bool_param(SPC_SDR_AGC_ENABLED, "AGC", true, SPC_SDR_GROUP_GAIN)
                .param_description("Automatic gain control")
            // Gain as a device-independent dB float (snapped to the nearest
            // hardware step when applied). Avoids the dynamic device-table enum,
            // whose options don't exist until the radio runs — which clamped a
            // saved gain index on every stopped save/load.
            .float_param(SPC_SDR_GAIN, "Manual Gain (dB)", 0.0f, 50.0f, 30.0f, 0.1f, SPC_SDR_GROUP_GAIN)
                .param_description("Manual tuner gain in dB, snapped to the nearest step the device supports (active when AGC is off)")
            // Hardware
            .enum_param(SPC_SDR_DIRECT_SAMPLING, "Direct Sampling", {"Off", "I-ADC", "Q-ADC"}, 0, SPC_SDR_GROUP_HARDWARE)
                .param_description("Direct sampling mode for HF reception below 24 MHz (V3 only)")
            .bool_param("offset_tuning", "Offset Tuning", false, SPC_SDR_GROUP_HARDWARE)
                .param_description("Avoid DC spike at center frequency for zero-IF tuners")
            .bool_param(SPC_SDR_BIAS_TEE, "Bias-T", false, SPC_SDR_GROUP_HARDWARE)
                .param_description("Enable bias-T voltage output on GPIO 0 for active antennas/LNAs")
            .int_param(SPC_SDR_FREQ_CORRECTION, "Freq Correction (PPM)", -100, 100, 0, 1, SPC_SDR_GROUP_HARDWARE)
                .param_description("Frequency correction in parts per million for crystal offset")
            .bool_param(SPC_SDR_DITHERING, "Frequency Dithering", true, SPC_SDR_GROUP_HARDWARE)
                .param_description("Frequency dithering to improve phase noise on R828D tuner")
            .bool_param("test_mode", "Test Mode", false, SPC_SDR_GROUP_HARDWARE)
                .param_description("Output 8-bit counter instead of samples (debug)")
            .int_param("if_gain", "IF Gain (0.1 dB)", -30, 90, 0, 10, SPC_SDR_GROUP_HARDWARE)
                .param_description("Intermediate frequency gain in tenths of dB (E4000 tuner only)")
            .streaming().device_scan()
            .build_into(g_desc);

        scan_devices(nullptr);
        g_desc_initialized = true;
    }

    return &g_desc;
}

// ── lifecycle ───────────────────────────────────────────────────────

static SpcPluginInstance* create_instance()
{
    auto* s = new RtlSdrState{};
    s->batch_buffer.resize(8192 * 2);

    auto* desc = get_descriptor();
    spc::sdr::init_iq_table(s->output_table, &desc->ports[0].schema);

    return reinterpret_cast<SpcPluginInstance*>(s);
}

static void destroy_instance(SpcPluginInstance* inst)
{
    auto* s = state(inst);
    s->device.reset();
    spc_table_free(&s->output_table);
    delete s;
}

// ── parameters ──────────────────────────────────────────────────────

static int set_parameter(SpcPluginInstance* inst, const char* name,
                         const SpcParameterDesc* value)
{
    auto* s = state(inst);
    auto* dev = s->device.get();
    bool live = dev && dev->is_open();

    if (spc::try_set_enum(name, value, "device", s->device_idx)) return 0;

    if (spc::try_set_int(name, value, SPC_SDR_CENTER_FREQ, s->center_freq)) {
        if (live) {
            dev->set_center_freq(static_cast<uint32_t>(s->center_freq));
            // V3: auto-toggle direct sampling for HF, matching start() behavior
            // V4 uses a built-in upconverter and handles HF transparently
            if (!dev->is_v4()) {
                if (s->center_freq < 24000000)
                    dev->set_direct_sampling(2);   // Q-ADC for HF
                else if (s->direct_sampling == 0)
                    dev->set_direct_sampling(0);   // restore normal tuner mode
            }
        }
        return 0;
    }
    if (spc::try_set_int(name, value, SPC_SDR_SAMPLE_RATE, s->sample_rate)) {
        s->actual_sample_rate = static_cast<double>(s->sample_rate);
        if (live) dev->set_sample_rate(static_cast<uint32_t>(s->sample_rate));
        return 0;
    }
    if (spc::try_set_int(name, value, SPC_SDR_BANDWIDTH, s->bandwidth)) {
        if (live) dev->set_bandwidth(static_cast<uint32_t>(s->bandwidth));
        return 0;
    }
    if (spc::try_set_bool(name, value, SPC_SDR_AGC_ENABLED, s->agc_enabled)) {
        if (live) {
            dev->set_agc(s->agc_enabled != 0);
            dev->set_tuner_gain_mode(s->agc_enabled == 0); // manual when AGC off
            if (!s->agc_enabled) apply_manual_gain(dev, s->gain_db, &s->host.cached_log);
        }
        return 0;
    }
    if (spc::try_set_float(name, value, SPC_SDR_GAIN, s->gain_db)) {
        if (live && !s->agc_enabled) apply_manual_gain(dev, s->gain_db, &s->host.cached_log);
        else SPC_LOG_INFO(&s->host.cached_log, "RTL-SDR: gain %.1f dB stored, not applied (live=%d agc=%d)",
                          static_cast<double>(s->gain_db), static_cast<int>(live), static_cast<int>(s->agc_enabled));
        return 0;
    }
    if (spc::try_set_enum(name, value, SPC_SDR_DIRECT_SAMPLING, s->direct_sampling)) {
        if (live) dev->set_direct_sampling(s->direct_sampling);
        return 0;
    }
    if (spc::try_set_bool(name, value, "offset_tuning", s->offset_tuning)) {
        if (live) dev->set_offset_tuning(s->offset_tuning != 0);
        return 0;
    }
    if (spc::try_set_bool(name, value, SPC_SDR_BIAS_TEE, s->bias_tee)) {
        if (live) dev->set_bias_tee(s->bias_tee != 0);
        return 0;
    }
    if (spc::try_set_int(name, value, SPC_SDR_FREQ_CORRECTION, s->freq_correction)) {
        if (live) dev->set_freq_correction(s->freq_correction);
        return 0;
    }
    if (spc::try_set_bool(name, value, SPC_SDR_DITHERING, s->dithering)) {
        if (live) dev->set_dithering(s->dithering != 0);
        return 0;
    }
    if (spc::try_set_bool(name, value, "test_mode", s->test_mode)) {
        if (live) dev->set_testmode(s->test_mode != 0);
        return 0;
    }
    if (spc::try_set_int(name, value, "if_gain", s->if_gain)) {
        if (live) dev->set_tuner_if_gain(1, s->if_gain);
        return 0;
    }

    return SPC_ERR_NOT_FOUND;
}

static int get_parameter(SpcPluginInstance* inst, const char* name,
                         SpcParameterDesc* out)
{
    auto* s = state(inst);
    bool no_dev = (s->device_idx == 0);
    bool agc_on = (s->agc_enabled != 0);

    // dynamic device enum — populate labels from registry
    if (std::strcmp(name, "device") == 0) {
        out->type = SPC_PARAM_ENUM;
        out->enum_val.value = s->device_idx;
        out->enum_val.count = g_registry.count;
        for (int i = 0; i < g_registry.count; ++i)
            std::strncpy(out->enum_val.labels[i], g_registry.devices[i].label, SPC_PARAM_ENUM_LABEL_MAX);
        return 0;
    }

    // tuning
    if (spc::try_get_int(name, out, SPC_SDR_CENTER_FREQ, s->center_freq)) {
        if (no_dev) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_int(name, out, SPC_SDR_SAMPLE_RATE, s->sample_rate)) {
        if (no_dev) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_int(name, out, SPC_SDR_BANDWIDTH, s->bandwidth)) {
        if (no_dev) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }

    // gain
    if (spc::try_get_bool(name, out, SPC_SDR_AGC_ENABLED, s->agc_enabled)) {
        if (no_dev) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_float(name, out, SPC_SDR_GAIN, s->gain_db)) {
        if (no_dev || agc_on) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }

    // hardware
    if (spc::try_get_enum(name, out, SPC_SDR_DIRECT_SAMPLING, s->direct_sampling)) {
        if (no_dev) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_bool(name, out, "offset_tuning", s->offset_tuning)) {
        if (no_dev) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_bool(name, out, SPC_SDR_BIAS_TEE, s->bias_tee)) {
        if (no_dev) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_int(name, out, SPC_SDR_FREQ_CORRECTION, s->freq_correction)) {
        if (no_dev) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_bool(name, out, SPC_SDR_DITHERING, s->dithering)) {
        if (no_dev) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_bool(name, out, "test_mode", s->test_mode)) {
        if (no_dev) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }
    if (spc::try_get_int(name, out, "if_gain", s->if_gain)) {
        if (no_dev) out->flags |= SPC_PARAM_FLAG_DISABLED;
        return 0;
    }

    return SPC_ERR_NOT_FOUND;
}

// ── streaming ───────────────────────────────────────────────────────

static int start(SpcPluginInstance* inst)
{
    auto* s = state(inst);
    s->sample_count = 0;

    if (s->device_idx == 0) {
        SPC_LOG_INFO(&s->host.cached_log, "RTL-SDR: no device selected");
        return 0;
    }

    if (!spc::rtlsdr::RtlSdrDevice::is_api_loaded()) {
        SPC_LOG_ERROR(&s->host.cached_log, "rtlsdr library not available");
        return -1;
    }

    uint32_t hw_index = g_registry.device_indices[s->device_idx];
    s->device = std::make_unique<spc::rtlsdr::RtlSdrDevice>(&s->host.cached_log);
    if (!s->device->open(hw_index)) {
        s->device.reset();
        return -1;
    }

    // query the device's supported gain steps (used to snap the dB gain)
    g_registry.gain_count = s->device->query_tuner_gains(g_registry.gains, 64);

    // apply all parameters
    uint32_t rate = static_cast<uint32_t>(s->sample_rate);
    s->device->set_sample_rate(rate);
    s->actual_sample_rate = static_cast<double>(rate);

    s->device->set_center_freq(static_cast<uint32_t>(s->center_freq));
    s->device->set_bandwidth(static_cast<uint32_t>(s->bandwidth));
    s->device->set_freq_correction(s->freq_correction);
    // V3: auto-enable direct sampling for HF (<24 MHz); V4 uses upconverter
    if (!s->device->is_v4() && s->center_freq < 24000000)
        s->device->set_direct_sampling(2);
    else
        s->device->set_direct_sampling(s->direct_sampling);
    s->device->set_offset_tuning(s->offset_tuning != 0);
    s->device->set_bias_tee(s->bias_tee != 0);
    s->device->set_testmode(s->test_mode != 0);
    s->device->set_dithering(s->dithering != 0);
    if (s->if_gain != 0)
        s->device->set_tuner_if_gain(1, s->if_gain);

    if (s->agc_enabled) {
        s->device->set_agc(true);
        s->device->set_tuner_gain_mode(false); // auto
    } else {
        s->device->set_agc(false);
        s->device->set_tuner_gain_mode(true); // manual
        apply_manual_gain(s->device.get(), s->gain_db, &s->host.cached_log);
    }

    if (!s->device->start_streaming()) {
        s->device.reset();
        return -1;
    }

    s->streaming = true;
    SPC_LOG_INFO(&s->host.cached_log, "RTL-SDR started (freq=%d Hz, rate=%u Hz, %s)",
                 s->center_freq, rate, s->device->is_v4() ? "V4" : "V3");
    return 0;
}

static int stop(SpcPluginInstance* inst)
{
    auto* s = state(inst);
    s->streaming = false;
    if (s->device) {
        // close() disables bias-T (guarded) before releasing the device, so a
        // librtlsdr GPIO-write fault under rapid start/stop can't skip the
        // release and wedge the next open.
        s->device->close();
        s->device.reset();
    }
    SPC_LOG_INFO(&s->host.cached_log, "RTL-SDR stopped (%llu samples)",
                 static_cast<unsigned long long>(s->sample_count));
    return 0;
}

// ── process ─────────────────────────────────────────────────────────

static constexpr uint32_t BATCH_SIZE = 4096;

static int process(SpcPluginInstance* inst, const SpcData*, uint32_t,
                   SpcData* outputs, uint32_t output_count)
{
    auto* s = state(inst);
    if (output_count < 1) return -1;

    if (!s->device || !s->streaming) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return 0;
    }

    uint32_t batch = s->device->read_iq(s->batch_buffer.data(), BATCH_SIZE);
    if (batch == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return 0;
    }

    if (spc_table_resize(&s->output_table, batch) != 0) return -1;
    std::memcpy(s->output_table.data, s->batch_buffer.data(),
                static_cast<size_t>(batch) * 4);

    auto ts = spc::clock::now_utc_ns(s->host);

    spc::sdr::set_iq_metadata(
        s->output_table,
        s->actual_sample_rate,
        static_cast<double>(s->center_freq),
        s->actual_sample_rate, // bandwidth ≈ sample rate for RTL-SDR
        s->agc_enabled ? 0.0 : static_cast<double>(s->gain_db), // manual gain in dB (0 = AGC)
        s->agc_enabled != 0,
        8, // RTL-SDR native bit depth
        s->sample_count / BATCH_SIZE,
        ts
    );

    outputs[0].type = SPC_DATA_SIGNAL;
    outputs[0].table = &s->output_table;

    s->sample_count += batch;
    return 0;
}

// ── mandatory-parameter validation ──────────────────────────────────
// Device index 0 is the "(no devices found)" placeholder; user must scan
// and pick a real device before starting.
static int validate_mandatory(SpcPluginInstance* inst, SpcMissingParams* out)
{
    auto* s = state(inst);
    if (s->device_idx <= 0) {
        spc::add_missing(*out, "device");
    }
    return 0;
}

// ── export ──────────────────────────────────────────────────────────

SPC_PLUGIN_VTABLE(
    .get_descriptor     = get_descriptor,
    .create_instance    = create_instance,
    .destroy_instance   = destroy_instance,
    .set_parameter      = set_parameter,
    .get_parameter      = get_parameter,
    .process            = process,
    .start              = start,
    .stop               = stop,
    .set_host_services  = set_host_services,
    .scan_devices       = scan_devices,
    .validate_mandatory = validate_mandatory
)
