#include "nesdr_models.h"
#include "rtl_sdr_device.h"

#include <speculor/sdr_source_helpers.h>
#include <speculor/sdr_params.h>
#include <spc_clock.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <thread>

using spc::nesdr::BiasTee;
using spc::nesdr::ModelCaps;
using spc::nesdr::Tuner;

// ── device registry ─────────────────────────────────────────────────

static struct NesdrRegistry {
    spc::sdr::DeviceEntry devices[spc::sdr::MAX_DEVICES];
    char ids[spc::sdr::MAX_DEVICES][SPC_PARAM_ENUM_ID_MAX];  // persistence keys
    uint32_t device_indices[spc::sdr::MAX_DEVICES];  // enum index -> hw index
    const ModelCaps* caps[spc::sdr::MAX_DEVICES];    // USB-string match, may be null
    int count = 0;

    // set when two dongles report the same serial, which leaves the
    // enumeration index as the only thing telling them apart
    bool ambiguous_serials = false;

    bool initialized = false;

    void scan(SpcLogContext* log = nullptr)
    {
        count = 0;
        ambiguous_serials = false;

        devices[count].index = -1;
        std::strncpy(devices[count].label, "None", SPC_PARAM_ENUM_LABEL_MAX);
        ids[count][0] = '\0';
        device_indices[count] = UINT32_MAX;
        caps[count] = nullptr;
        count++;

        if (!spc::rtlsdr::RtlSdrDevice::load_api(log)) {
            initialized = true;
            return;
        }

        // Every RTL2832U dongle is listed, not just the ones identifiable as
        // NooElec: the Mini and Nano bodies frequently ship with stock Realtek
        // EEPROM strings, and filtering on the manufacturer string would show
        // those owners an empty list for a radio that is plugged in.
        auto hw = spc::rtlsdr::RtlSdrDevice::enumerate();
        for (const auto& dev : hw) {
            if (count >= spc::sdr::MAX_DEVICES) break;
            const ModelCaps* m = spc::nesdr::resolve_by_usb(dev.manufacturer, dev.product);
            const char* name = m ? m->label : dev.name;

            // Stock RTL2832U bodies — which the Mini and Nano often are — keep
            // the factory serial 00000001, so two of them label identically.
            // That is unreadable, and it is also indistinguishable to the
            // project loader, which resolves a saved option by label and would
            // point both nodes at one radio. Fall back to the enumeration
            // index, the same -d the rtl_test / rtl_eeprom tools take.
            const bool clash = std::count_if(hw.begin(), hw.end(), [&](const auto& d2) {
                const ModelCaps* m2 = spc::nesdr::resolve_by_usb(d2.manufacturer, d2.product);
                return std::strcmp(d2.serial, dev.serial) == 0 &&
                       std::strcmp(m2 ? m2->label : d2.name, name) == 0;
            }) > 1;
            if (clash) ambiguous_serials = true;

            devices[count].index = static_cast<int>(dev.index);
            if (clash)
                std::snprintf(devices[count].label, SPC_PARAM_ENUM_LABEL_MAX,
                              "#%u %s", dev.index, name);
            else
                std::snprintf(devices[count].label, SPC_PARAM_ENUM_LABEL_MAX,
                              "%s [%s]", name, dev.serial);

            // Persistence key, preferred over the label by the project loader.
            // The serial alone will not do for the reason above; pairing it
            // with the index makes it unique for as long as the dongles stay
            // in the ports they were scanned in.
            std::snprintf(ids[count], SPC_PARAM_ENUM_ID_MAX, "%s@%u",
                          dev.serial, dev.index);

            device_indices[count] = dev.index;
            caps[count] = m;
            count++;
        }

        initialized = true;
    }
} g_registry;

// ── state ───────────────────────────────────────────────────────────

struct NesdrState {
    spc::HostServices host;

    std::unique_ptr<spc::rtlsdr::RtlSdrDevice> device;

    // Device
    int32_t device_idx = 0;

    // Tuning
    int32_t center_freq = 100000000;
    int32_t sample_rate = 2048000;
    int32_t bandwidth = 0;  // 0 = auto

    // Gain
    int32_t agc_enabled = 1;
    float   gain_db = 30.0f;

    // Hardware
    int32_t model_idx = 0;  // 0 = auto-detect, else model_at(model_idx - 1)
    int32_t direct_sampling = 0;
    int32_t applied_ds = -1;  // -1 = nothing programmed yet
    int32_t offset_tuning = 0;
    int32_t bias_tee = 0;
    int32_t freq_correction = 0;
    int32_t test_mode = 0;
    int32_t dithering = 1;
    int32_t if_gain_stage = 0;  // enum index; hardware stage is index + 1
    int32_t if_gain = 0;        // tenths of dB

    // Profile in force. Before start it is the pre-open resolution (override or
    // USB strings); start() replaces it with the tuner-confirmed one.
    const ModelCaps* profile = nullptr;

    // Gain steps of this instance's device, queried at open. Per instance and
    // not shared: two nodes driving two dongles can be on different tuners (an
    // R820T2 and an E4000 snap to different steps).
    int gains[64];
    int gain_count = 0;

    // What the hardware reported it actually did, stamped into the I/Q
    // metadata. The tuner snaps to what its PLL can synthesise, so the request
    // is not the truth.
    double actual_sample_rate = 2048000.0;
    double actual_center_freq = 100000000.0;

    // Deduplicate the clamp / L-band-gap warnings, which would otherwise fire
    // on every retune of a sweep.
    int32_t warned_freq = 0;

    std::vector<int16_t> batch_buffer;
    uint64_t sample_count = 0;
    bool streaming = false;
};

SPC_PLUGIN_CAST(NesdrState)
SPC_PLUGIN_HOST_SERVICES(NesdrState, host)

// ── profile resolution ──────────────────────────────────────────────

// The profile to use before the device is open: an explicit model override
// wins, then the USB-string match from the scan. Falling back to the R820T2
// generic rather than to nothing keeps the parameter gating deterministic for
// an unidentified dongle; start() corrects it from the tuner read.
static const ModelCaps* pre_open_profile(const NesdrState* s)
{
    if (s->model_idx > 0) {
        if (const ModelCaps* m = spc::nesdr::model_at(s->model_idx - 1)) return m;
    }
    if (s->device_idx > 0 && s->device_idx < g_registry.count) {
        if (const ModelCaps* m = g_registry.caps[s->device_idx]) return m;
    }
    return spc::nesdr::model_at(0);  // generic_r820t
}

static const ModelCaps* active_profile(const NesdrState* s)
{
    return s->profile ? s->profile : pre_open_profile(s);
}

// ── descriptor & device scan ────────────────────────────────────────

static spc::DescriptorStorage g_desc;
static bool g_desc_initialized = false;

// Fill the model override's options from the capability table rather than
// restating them as literals, so a model added to the table cannot go missing
// from the parameter.
static void patch_model_enum()
{
    const uint32_t index = spc::param_index(g_desc.get(), "model");
    if (index == UINT32_MAX) return;
    SpcParameterDesc p;
    spc::param_carrier(g_desc.get()->params[index], p);
    auto& ev = p.enum_val;
    int n = 1 + spc::nesdr::model_count();
    if (n > SPC_PARAM_ENUM_MAX) n = SPC_PARAM_ENUM_MAX;
    ev.count = n;
    std::strncpy(ev.labels[0], "Auto-detect", SPC_PARAM_ENUM_LABEL_MAX - 1);
    ev.labels[0][SPC_PARAM_ENUM_LABEL_MAX - 1] = '\0';
    for (int i = 1; i < n; ++i) {
        const ModelCaps* m = spc::nesdr::model_at(i - 1);
        std::strncpy(ev.labels[i], m ? m->label : "?", SPC_PARAM_ENUM_LABEL_MAX - 1);
        ev.labels[i][SPC_PARAM_ENUM_LABEL_MAX - 1] = '\0';
    }
    g_desc.set_param(index, p);
}

static void scan_devices(SpcHost* svc)
{
    SpcLogContext log{};
    if (svc && svc->log) log = spc::log_context(svc);

    SPC_LOG_INFO(&log, "NESDR: scanning for devices...");
    g_registry.scan(&log);

    if (!spc::rtlsdr::RtlSdrDevice::is_api_loaded()) {
        SPC_LOG_WARN(&log, "NESDR: rtlsdr library not found");
    } else if (g_registry.count <= 1) {
#ifdef _WIN32
        SPC_LOG_WARN(&log, "NESDR: no devices found — check the dongle is "
                           "plugged in and visible in Device Manager.");
#else
        SPC_LOG_WARN(&log, "NESDR: no devices found — check the dongle is "
                           "plugged in ('lsusb' should list it, typically "
                           "ID 0bda:2838).");
#endif
    } else {
        SPC_LOG_INFO(&log, "NESDR: found %d device(s)", g_registry.count - 1);
        for (int i = 1; i < g_registry.count; ++i)
            SPC_LOG_INFO(&log, "NESDR:   [%d] %s%s", i, g_registry.devices[i].label,
                         g_registry.caps[i] ? "" : "  (model not identified from USB strings"
                                                   " — set Model by hand if the defaults are wrong)");

        if (g_registry.ambiguous_serials)
            SPC_LOG_WARN(&log, "NESDR: two or more dongles report the same serial "
                               "number, so the only thing telling them apart is the "
                               "#N enumeration index now shown in the device list — "
                               "and that index follows the USB port, so it moves if "
                               "they are replugged. Flash a unique serial to each "
                               "instead: 'rtl_eeprom -d 1 -s 00000002', then replug.");
    }
}

static const SpcNodeInfo* get_descriptor()
{
    if (!g_desc_initialized) {
        spc::DescriptorBuilder("nesdr", "NooElec NESDR", "Signal/SDR/Sources")
            .author("Speculor").version("0.1.0")
            .source()
            .help("Streams I/Q data from NooElec NESDR receivers (Mini, Nano, SMArt, "
                  "SMArTee and XTR bodies), gating each control on what the selected "
                  "model's board actually provides")
            // Not yet run against any NESDR: the capability table is built from
            // NooElec's published specifications, and only the table itself is
            // covered by tests. PREVIEW would claim "functional".
            .maturity(SPC_MATURITY_EXPERIMENTAL)
            .tags({"radio"})
            .output_signal("iq_out", "I/Q Output", {
                {"i", SPC_FIELD_INT16},
                {"q", SPC_FIELD_INT16},
            })
            // Device
            .enum_param("device", "Device", {"(no devices found)"}, 0, SPC_SDR_GROUP_DEVICE).listed()
                .param_help("Connected receiver to use. Every RTL2832U device is listed; "
                            "ones whose USB strings identify a NESDR model are named")
                .mandatory()
            // Tuning — the range is the union across the line, since a
            // descriptor is static while the profile is not. What the selected
            // model cannot reach is clamped when applied, and the I/Q metadata
            // reports where the tuner actually landed.
            .int_param(SPC_SDR_CENTER_FREQ, "Center Freq (Hz)", 100000,
                       static_cast<int32_t>(spc::nesdr::MAX_PARAM_FREQ_HZ),
                       100000000, 1000, SPC_SDR_GROUP_TUNING)
                .param_help("Receiver center frequency in Hz, clamped to the selected "
                            "model's range (100 kHz only on the SMArt v5, which has a "
                            "wired Q-branch)")
            .int_param(SPC_SDR_SAMPLE_RATE, "Sample Rate (Hz)", 225001, 3200000, 2048000, 1000, SPC_SDR_GROUP_TUNING)
                .param_help("ADC sample rate in Hz (higher = wider bandwidth, more CPU)")
            .int_param(SPC_SDR_BANDWIDTH, "IF Bandwidth (Hz)", 0, 3200000, 0, 10000, SPC_SDR_GROUP_TUNING)
                .param_help("IF bandwidth in Hz (0 = automatic, matched to sample rate)")
            // Gain
            .bool_param(SPC_SDR_AGC_ENABLED, "AGC", true, SPC_SDR_GROUP_GAIN)
                .param_help("Automatic gain control")
            .float_param(SPC_SDR_GAIN, "Manual Gain (dB)", -1.0f, 50.0f, 30.0f, 0.1f, SPC_SDR_GROUP_GAIN)
                .param_help("Manual tuner gain in dB, snapped to the nearest step the "
                            "device supports (R820T2 spans 0-49.6 dB, E4000 -1.0-42.0 dB). "
                            "Active when AGC is off")
            // Hardware
            .enum_param("model", "Model", {"Auto-detect"}, 0, SPC_SDR_GROUP_HARDWARE)
                .param_help("Which NESDR board this is, which decides what the controls "
                            "below can do. Auto-detect reads the USB strings and confirms "
                            "the tuner family when the device opens; set it by hand for a "
                            "dongle flashed with stock Realtek strings")
            .enum_param(SPC_SDR_DIRECT_SAMPLING, "Direct Sampling", {"Off", "I-ADC", "Q-ADC"}, 0, SPC_SDR_GROUP_HARDWARE)
                .param_help("Direct sampling for HF below 24 MHz. Only meaningful on a "
                            "board with a wired Q-branch (SMArt v5); every other NESDR "
                            "needs an upconverter, so this stays disabled there")
            .bool_param("offset_tuning", "Offset Tuning", false, SPC_SDR_GROUP_HARDWARE)
                .param_help("Avoid the DC spike at center frequency on zero-IF tuners. "
                            "E4000 (XTR) only: on an R820T2 librtlsdr repurposes this "
                            "call to switch the bias tee, so it stays disabled there")
            .bool_param(SPC_SDR_BIAS_TEE, "Bias-T", false, SPC_SDR_GROUP_HARDWARE)
                .param_help("Bias-T voltage for active antennas and LNAs. Disabled on "
                            "boards without the circuit; on a SMArTee it is powered by "
                            "hardware whenever the dongle is, and cannot be switched off")
            .int_param(SPC_SDR_FREQ_CORRECTION, "Freq Correction (PPM)", -100, 100, 0, 1, SPC_SDR_GROUP_HARDWARE)
                .param_help("Crystal offset correction in parts per million. TCXO models "
                            "(0.5 PPM) need approximately 0; the plain-crystal Mini 2 and "
                            "Nano 2 drift and do need it")
            .bool_param(SPC_SDR_DITHERING, "Frequency Dithering", true, SPC_SDR_GROUP_HARDWARE)
                .param_help("R820T/R820T2 only. On spreads the fractional-N PLL's spurs "
                            "into a noise pedestal, which is what general reception wants. "
                            "Off leaves discrete spurs but holds the tuner's phase steady, "
                            "which narrowband and phase-sensitive work needs")
            .bool_param("test_mode", "Test Mode", false, SPC_SDR_GROUP_HARDWARE)
                .param_help("Output an 8-bit counter instead of samples (debug)")
            .enum_param("if_gain_stage", "IF Gain Stage",
                        {"Stage 1", "Stage 2", "Stage 3", "Stage 4", "Stage 5", "Stage 6"},
                        0, SPC_SDR_GROUP_HARDWARE)
                .param_help("Which of the E4000's six IF gain stages the IF Gain value "
                            "below applies to (XTR models only)")
            .int_param("if_gain", "IF Gain (0.1 dB)", -30, 90, 0, 10, SPC_SDR_GROUP_HARDWARE)
                .param_help("Intermediate frequency gain in tenths of dB for the selected "
                            "stage (E4000 tuner, XTR models only)")
            .build_into(g_desc);

        patch_model_enum();
        scan_devices(nullptr);
        g_desc_initialized = true;
    }

    return g_desc.get();
}

// ── hardware application ────────────────────────────────────────────

// Snap a requested dB gain to the nearest step the tuner supports and apply it.
// The user-facing value stays a device-independent dB float so it persists
// across a device change; the snap happens here, against the table queried at
// open.
static void apply_manual_gain(NesdrState* s, float gain_db)
{
    int target = static_cast<int>(gain_db * 10.0f + (gain_db >= 0.0f ? 0.5f : -0.5f));
    if (s->gain_count > 0) {
        int best = s->gains[0];
        int best_d = target - best; if (best_d < 0) best_d = -best_d;
        for (int i = 1; i < s->gain_count; ++i) {
            int d = target - s->gains[i]; if (d < 0) d = -d;
            if (d < best_d) { best_d = d; best = s->gains[i]; }
        }
        target = best;
    }
    s->device->set_tuner_gain(target);
}

// Apply the AGC/manual gain configuration. When enabling AGC, the tuner's
// LNA/mixer gain-code registers are SEEDED with the manual gain first: the
// driver's auto branch never writes those code bits, and a cold start leaves
// them at the init array's minimum — AGC-on from cold then sits at minimum RF
// gain with the digital AGC amplifying the quantisation floor, and stays there
// until a manual gain is set once. Seeding makes AGC-on deterministic.
static void apply_gain_config(NesdrState* s)
{
    auto* dev = s->device.get();
    if (!dev) return;
    if (s->agc_enabled) {
        dev->set_tuner_gain_mode(true);
        apply_manual_gain(s, s->gain_db);
        dev->set_tuner_gain_mode(false);  // auto, keeping the seeded codes
        dev->set_agc(true);
    } else {
        dev->set_agc(false);
        dev->set_tuner_gain_mode(true);
        apply_manual_gain(s, s->gain_db);
    }
}

// Program the direct-sampling mode only when it actually changes, and repair
// what the driver's tuner re-init destroys. librtlsdr RE-INITIALISES THE WHOLE
// TUNER on every set_direct_sampling call — even a no-op "off" — silently
// resetting the bandwidth filter and the gain registers.
static void apply_direct_sampling(NesdrState* s, int want)
{
    if (want == s->applied_ds || !s->device) return;
    s->device->set_direct_sampling(want);
    s->applied_ds = want;
    s->device->set_bandwidth(static_cast<uint32_t>(s->bandwidth));
    apply_gain_config(s);
}

// The direct-sampling mode the profile allows for the current frequency. A
// board with no wired Q-branch is never switched into direct sampling at all:
// doing so on an HF retune would tune the receiver to a pin connected to
// nothing while re-initialising the tuner on the way.
static int wanted_direct_sampling(const NesdrState* s)
{
    const ModelCaps* p = active_profile(s);
    if (!p || !p->direct_sampling) return 0;
    if (p->auto_hf_direct &&
        static_cast<uint32_t>(s->center_freq) < spc::nesdr::HF_DIRECT_HZ) return 2;
    return s->direct_sampling;
}

// Clamp a requested frequency into the profile's range, warning once per
// distinct request so a sweep does not fill the log.
static uint32_t clamp_center_freq(NesdrState* s)
{
    const ModelCaps* p = active_profile(s);
    uint32_t want = static_cast<uint32_t>(s->center_freq);
    if (!p) return want;

    // Direct sampling reaches below the tuner's own floor by bypassing it.
    const bool hf = p->direct_sampling && wanted_direct_sampling(s) != 0;
    const uint32_t lo = hf ? 0u : p->freq_min_hz;
    uint32_t hi = p->freq_max_hz;
    if (hi > spc::nesdr::MAX_PARAM_FREQ_HZ) hi = spc::nesdr::MAX_PARAM_FREQ_HZ;

    const bool fresh = (s->warned_freq != s->center_freq);
    if (want < lo) {
        if (fresh) SPC_LOG_WARN(&s->host.cached_log,
            "NESDR: %u Hz is below the %s floor of %u Hz — tuning there instead",
            want, p->label, lo);
        s->warned_freq = s->center_freq;
        return lo;
    }
    if (want > hi) {
        if (fresh) SPC_LOG_WARN(&s->host.cached_log,
            "NESDR: %u Hz is above the %s ceiling of %u Hz — tuning there instead",
            want, p->label, hi);
        s->warned_freq = s->center_freq;
        return hi;
    }
    if (p->gap_lo_hz && want >= p->gap_lo_hz && want <= p->gap_hi_hz) {
        // Inside the E4000's L-band gap the PLL will not lock. Its exact edges
        // vary between individual tuners, so this is reported rather than
        // corrected — the request is passed through and the metadata will show
        // where the tuner actually ended up.
        if (fresh) SPC_LOG_WARN(&s->host.cached_log,
            "NESDR: %u Hz falls in the E4000 L-band gap (%u-%u Hz) — the tuner "
            "will not lock here",
            want, p->gap_lo_hz, p->gap_hi_hz);
        s->warned_freq = s->center_freq;
    }
    return want;
}

// Read back what the tuner actually did. Both are cached driver-side values,
// so this costs no USB traffic — but it is still done on apply rather than per
// batch, since a source feeding many consumers runs process() constantly.
static void refresh_actuals(NesdrState* s)
{
    if (!s->device) return;
    const uint32_t f = s->device->get_center_freq();
    const uint32_t r = s->device->get_sample_rate();
    if (f) s->actual_center_freq = static_cast<double>(f);
    if (r) s->actual_sample_rate = static_cast<double>(r);
}

static void apply_center_freq(NesdrState* s)
{
    if (!s->device) return;
    s->device->set_center_freq(clamp_center_freq(s));
    refresh_actuals(s);
}

// Grey out the controls the profile in force has no hardware behind. A bias tee
// the board powers by itself reads as on.
static void show_profile(NesdrState* s)
{
    const SpcNodeInfo* d = get_descriptor();
    const ModelCaps* p = active_profile(s);
    const bool e4000 = p && p->tuner == Tuner::E4000;

    s->host.set_disabled(spc::param_index(d, SPC_SDR_DIRECT_SAMPLING), !p || !p->direct_sampling);
    s->host.set_disabled(spc::param_index(d, "offset_tuning"), !p || !p->offset_tuning);
    s->host.set_disabled(spc::param_index(d, SPC_SDR_BIAS_TEE), !p || p->bias != BiasTee::Gpio);
    s->host.set_disabled(spc::param_index(d, SPC_SDR_DITHERING), !p || !p->dithering);
    s->host.set_disabled(spc::param_index(d, "if_gain_stage"), !e4000);
    s->host.set_disabled(spc::param_index(d, "if_gain"), !e4000);

    if (p && p->bias == BiasTee::AlwaysOn && !s->bias_tee) {
        s->bias_tee = 1;
        s->host.set_int(spc::param_index(d, SPC_SDR_BIAS_TEE), 1);
    }
}

// ── lifecycle ───────────────────────────────────────────────────────

static SpcPluginInstance* create_instance()
{
    auto* s = new NesdrState{};
    s->batch_buffer.resize(8192 * 2);

    return reinterpret_cast<SpcPluginInstance*>(s);
}

static void destroy_instance(SpcPluginInstance* inst)
{
    auto* s = state(inst);
    s->device.reset();
    delete s;
}

// ── parameters ──────────────────────────────────────────────────────

static int set_parameter(SpcPluginInstance* inst, const char* name,
                         const SpcParameterDesc* value)
{
    auto* s = state(inst);
    auto* dev = s->device.get();
    bool live = dev && dev->is_open();

    if (spc::try_set_enum(name, value, "device", s->device_idx)
        || spc::try_set_enum(name, value, "model", s->model_idx)) {
        // Only meaningful while stopped: the profile decides what start()
        // programs. Refresh the pre-open resolution so the parameter gating
        // updates immediately in the UI.
        if (!live) {
            s->profile = pre_open_profile(s);
            show_profile(s);
        }
        return 0;
    }

    if (spc::try_set_int(name, value, SPC_SDR_CENTER_FREQ, s->center_freq)) {
        if (live) {
            apply_direct_sampling(s, wanted_direct_sampling(s));
            apply_center_freq(s);
        }
        return 0;
    }
    if (spc::try_set_int(name, value, SPC_SDR_SAMPLE_RATE, s->sample_rate)) {
        s->actual_sample_rate = static_cast<double>(s->sample_rate);
        if (live) {
            dev->set_sample_rate(static_cast<uint32_t>(s->sample_rate));
            refresh_actuals(s);
        }
        return 0;
    }
    if (spc::try_set_int(name, value, SPC_SDR_BANDWIDTH, s->bandwidth)) {
        if (live) dev->set_bandwidth(static_cast<uint32_t>(s->bandwidth));
        return 0;
    }
    if (spc::try_set_bool(name, value, SPC_SDR_AGC_ENABLED, s->agc_enabled)) {
        if (live) apply_gain_config(s);
        return 0;
    }
    if (spc::try_set_float(name, value, SPC_SDR_GAIN, s->gain_db)) {
        if (live && !s->agc_enabled) apply_manual_gain(s, s->gain_db);
        return 0;
    }
    // The capability-gated controls below check the profile BEFORE storing, so
    // a rejected set leaves no trace: sdr_params.h makes SPC_NOT_FOUND the
    // report for "this device does not have that", and a plugin that answers
    // "unsupported" while quietly keeping the value would hand the next start()
    // a setting the user was told was refused. The host is handed back the
    // value still in force. A set that asks for that same value is no request
    // and is passed over in silence: every parameter arrives once, with its
    // default, before the first start().
    const uint32_t param = spc::param_index(get_descriptor(), name);
    if (std::strcmp(name, SPC_SDR_DIRECT_SAMPLING) == 0) {
        const ModelCaps* p = active_profile(s);
        if (!p || !p->direct_sampling) {
            if (value->enum_val.value == s->direct_sampling) return SPC_NOT_FOUND;
            SPC_LOG_WARN(&s->host.cached_log,
                "NESDR: %s has no wired Q-branch, so direct sampling would tune to "
                "nothing — ignoring. Use an upconverter for HF on this model",
                p ? p->label : "this device");
            s->host.set_choice(param, s->direct_sampling);
            return SPC_NOT_FOUND;
        }
        if (!spc::try_set_enum(name, value, SPC_SDR_DIRECT_SAMPLING, s->direct_sampling))
            return SPC_INVALID;
        if (live) apply_direct_sampling(s, wanted_direct_sampling(s));
        return 0;
    }
    if (std::strcmp(name, "offset_tuning") == 0) {
        const ModelCaps* p = active_profile(s);
        if (!p || !p->offset_tuning) {
            // Not a harmless no-op to swallow: on an R820T/R828D librtlsdr
            // routes this call straight to the bias tee, so honouring it would
            // put voltage on the antenna port.
            if (value->bool_val.value == s->offset_tuning) return SPC_NOT_FOUND;
            SPC_LOG_WARN(&s->host.cached_log,
                "NESDR: offset tuning is not available on %s — ignoring",
                p ? p->label : "this device");
            s->host.set_int(param, s->offset_tuning);
            return SPC_NOT_FOUND;
        }
        if (!spc::try_set_bool(name, value, "offset_tuning", s->offset_tuning))
            return SPC_INVALID;
        if (live) dev->set_offset_tuning(s->offset_tuning != 0);
        return 0;
    }
    if (std::strcmp(name, SPC_SDR_BIAS_TEE) == 0) {
        const ModelCaps* p = active_profile(s);
        if (!p || p->bias == BiasTee::None) {
            if (value->bool_val.value == s->bias_tee) return SPC_NOT_FOUND;
            SPC_LOG_WARN(&s->host.cached_log, "NESDR: %s has no bias tee — ignoring",
                         p ? p->label : "this device");
            s->host.set_int(param, s->bias_tee);
            return SPC_NOT_FOUND;
        }
        if (p->bias == BiasTee::AlwaysOn) {
            s->bias_tee = 1;  // hardware-powered whenever the dongle is
            if (!value->bool_val.value) s->host.set_int(param, 1);
            return 0;
        }
        if (!spc::try_set_bool(name, value, SPC_SDR_BIAS_TEE, s->bias_tee))
            return SPC_INVALID;
        if (live) dev->set_bias_tee(s->bias_tee != 0);
        return 0;
    }
    if (std::strcmp(name, SPC_SDR_DITHERING) == 0) {
        const ModelCaps* p = active_profile(s);
        if (!p || !p->dithering) {
            if (value->bool_val.value != s->dithering) s->host.set_int(param, s->dithering);
            return SPC_NOT_FOUND;
        }
        if (!spc::try_set_bool(name, value, SPC_SDR_DITHERING, s->dithering))
            return SPC_INVALID;
        if (live) {
            dev->set_dithering(s->dithering != 0);
            apply_center_freq(s);  // only reaches the PLL at its next programming
        }
        return 0;
    }
    if (std::strcmp(name, "if_gain_stage") == 0 || std::strcmp(name, "if_gain") == 0) {
        const ModelCaps* p = active_profile(s);
        if (!p || p->tuner != Tuner::E4000) {
            if (value->type == SPC_PARAM_ENUM) {
                if (value->enum_val.value != s->if_gain_stage)
                    s->host.set_choice(param, s->if_gain_stage);
            } else if (value->int_val.value != s->if_gain) {
                s->host.set_int(param, s->if_gain);
            }
            return SPC_NOT_FOUND;
        }
        const bool ok = spc::try_set_enum(name, value, "if_gain_stage", s->if_gain_stage)
                     || spc::try_set_int(name, value, "if_gain", s->if_gain);
        if (!ok) return SPC_INVALID;
        if (live) dev->set_tuner_if_gain(s->if_gain_stage + 1, s->if_gain);
        return 0;
    }

    if (spc::try_set_int(name, value, SPC_SDR_FREQ_CORRECTION, s->freq_correction)) {
        if (live) {
            dev->set_freq_correction(s->freq_correction);
            refresh_actuals(s);
        }
        return 0;
    }
    if (spc::try_set_bool(name, value, "test_mode", s->test_mode)) {
        if (live) dev->set_testmode(s->test_mode != 0);
        return 0;
    }

    return SPC_NOT_FOUND;
}

static void param_rules(const SpcValue* values, uint32_t* states)
{
    spc::ParamRules r(get_descriptor(), values, states);
    // the first option is "None": the user must scan and pick a real device
    const bool no_dev = r.choice("device") <= 0;
    r.missing("device", no_dev);
    for (const char* name : {SPC_SDR_CENTER_FREQ, SPC_SDR_SAMPLE_RATE, SPC_SDR_BANDWIDTH,
                             SPC_SDR_AGC_ENABLED, SPC_SDR_DIRECT_SAMPLING, "offset_tuning",
                             SPC_SDR_BIAS_TEE, SPC_SDR_FREQ_CORRECTION, SPC_SDR_DITHERING,
                             "test_mode", "if_gain_stage", "if_gain"})
        r.disable(name, no_dev);
    r.disable(SPC_SDR_GAIN, no_dev || r.on(SPC_SDR_AGC_ENABLED));
}

static int list_options(uint32_t param, const SpcValue*, int refresh,
                        SpcHost* host_services,
                        SpcOptionSinkFn sink, void* sink_ctx)
{
    if (param != spc::param_index(get_descriptor(), "device")) return SPC_NOT_FOUND;
    if (refresh) scan_devices(host_services);
    for (int i = 0; i < g_registry.count; ++i)
        sink(sink_ctx, g_registry.ids[i], g_registry.devices[i].label);
    return SPC_OK;
}

// ── streaming ───────────────────────────────────────────────────────

// Settle the profile against the opened device. The tuner read is
// authoritative for capabilities — it reads the silicon rather than a string
// that rtl_eeprom can rewrite — so a disagreement is reported and the tuner
// wins, unless the user pinned a model by hand.
static void confirm_profile(NesdrState* s)
{
    const ModelCaps* pre = pre_open_profile(s);
    const ModelCaps* by_tuner = spc::nesdr::resolve_by_tuner(s->device->get_tuner_type());

    s->profile = pre;
    if (!by_tuner) {
        SPC_LOG_WARN(&s->host.cached_log,
            "NESDR: device reports an unrecognised tuner — keeping the %s profile",
            pre ? pre->label : "default");
    } else if (pre && pre->tuner != by_tuner->tuner) {
        if (s->model_idx > 0) {
            SPC_LOG_WARN(&s->host.cached_log,
                "NESDR: Model is pinned to %s (%s) but the device reports a %s tuner — "
                "honouring the pin; clear it to Auto-detect if reception is wrong",
                pre->label, pre->tuner_name, by_tuner->tuner_name);
        } else {
            SPC_LOG_WARN(&s->host.cached_log,
                "NESDR: USB strings suggest %s (%s) but the device reports a %s tuner — "
                "trusting the tuner",
                pre->label, pre->tuner_name, by_tuner->tuner_name);
            s->profile = by_tuner;
        }
    }

    const ModelCaps* p = s->profile;
    if (p)
        SPC_LOG_INFO(&s->host.cached_log,
            "NESDR: %s (tuner %s, profile %s, bias tee %s, direct sampling %s, %s)",
            p->label, p->tuner_name, p->id, spc::nesdr::bias_tee_name(p->bias),
            p->direct_sampling ? "available" : "not wired",
            p->tcxo ? "0.5 PPM TCXO" : "plain crystal");
}

static int start(SpcPluginInstance* inst)
{
    auto* s = state(inst);
    s->sample_count = 0;
    s->warned_freq = 0;

    if (s->device_idx == 0) {
        SPC_LOG_INFO(&s->host.cached_log, "NESDR: no device selected");
        return 0;
    }

    // A project restores its saved device by appending the option it recorded
    // when nothing in the current scan matches, so the index can point past
    // everything this registry knows — there is no hardware index behind it.
    if (s->device_idx >= g_registry.count) {
        SPC_LOG_ERROR(&s->host.cached_log,
                      "NESDR: the saved device is not connected — rescan and select one");
        return -1;
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

    confirm_profile(s);
    show_profile(s);
    const ModelCaps* p = s->profile;

    s->gain_count = s->device->query_tuner_gains(s->gains, 64);

    uint32_t rate = static_cast<uint32_t>(s->sample_rate);
    s->device->set_sample_rate(rate);

    // Direct sampling first: a freshly opened device is in normal tuner mode,
    // and a mode change re-initialises the tuner — anything set before it
    // (bandwidth, gain) would be silently undone.
    s->applied_ds = 0;
    apply_direct_sampling(s, wanted_direct_sampling(s));

    // Before the frequency: dithering takes hold at the next PLL programming,
    // so applying it afterwards leaves the tuner as it was until something
    // retunes.
    if (p && p->dithering) s->device->set_dithering(s->dithering != 0);
    apply_center_freq(s);
    s->device->set_bandwidth(static_cast<uint32_t>(s->bandwidth));
    s->device->set_freq_correction(s->freq_correction);
    if (p && p->offset_tuning) s->device->set_offset_tuning(s->offset_tuning != 0);
    if (p && p->bias == BiasTee::Gpio) s->device->set_bias_tee(s->bias_tee != 0);
    s->device->set_testmode(s->test_mode != 0);
    if (p && p->tuner == Tuner::E4000 && s->if_gain != 0)
        s->device->set_tuner_if_gain(s->if_gain_stage + 1, s->if_gain);

    apply_gain_config(s);
    refresh_actuals(s);

    if (!s->device->start_streaming()) {
        s->device.reset();
        return -1;
    }

    s->streaming = true;
    SPC_LOG_INFO(&s->host.cached_log, "NESDR started (freq=%.0f Hz, rate=%.0f Hz)",
                 s->actual_center_freq, s->actual_sample_rate);
    return 0;
}

static int stop(SpcPluginInstance* inst)
{
    auto* s = state(inst);
    s->streaming = false;
    if (s->device) {
        // close() disables bias-T (guarded) before releasing the device, so a
        // librtlsdr GPIO-write fault under rapid start/stop can't skip the
        // release and wedge the next open. On a SMArTee the bias tee is not on
        // that GPIO, so the write is inert and the supply stays up.
        s->device->close();
        s->device.reset();
    }
    s->profile = nullptr;  // re-resolve on the next start
    show_profile(s);
    SPC_LOG_INFO(&s->host.cached_log, "NESDR stopped (%llu samples)",
                 static_cast<unsigned long long>(s->sample_count));
    return 0;
}

// Phase-1 abort (engine two-phase contract): interrupt the blocking async read
// so stop() is prompt and never waits a read timeout. Runs concurrently with
// process(); only kicks librtlsdr's cancel, frees nothing.
static void request_stop(SpcPluginInstance* inst)
{
    auto* s = state(inst);
    if (s->device) s->device->request_stop_streaming();
}

// ── process ─────────────────────────────────────────────────────────

static constexpr uint32_t BATCH_SIZE = 4096;

static int process(SpcPluginInstance* inst, SpcStep* step)
{
    SpcData* outputs = step->outputs;
    const uint32_t output_count = step->output_count;
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

    SpcTable* out = s->host.table(0, batch);
    if (!out) return -1;
    std::memcpy(out->data, s->batch_buffer.data(),
                static_cast<size_t>(batch) * 4);

    auto ts = spc::clock::now_utc_ns(s->host);

    spc::sdr::set_iq_metadata(
        *out,
        s->actual_sample_rate,
        s->actual_center_freq,
        s->actual_sample_rate,  // bandwidth ~ sample rate for an RTL2832U
        s->agc_enabled ? 0.0 : static_cast<double>(s->gain_db),
        s->agc_enabled != 0,
        8,  // RTL2832U native bit depth
        s->sample_count / BATCH_SIZE,
        ts
    );

    outputs[0].type = SPC_DATA_SIGNAL;
    outputs[0].table = out;

    s->sample_count += batch;
    return 0;
}

// ── export ──────────────────────────────────────────────────────────

SPC_EXPORT_PLUGIN(
    .info               = get_descriptor,
    .create_instance    = create_instance,
    .destroy_instance   = destroy_instance,
    .params_changed    = spc::params_by_name<get_descriptor, set_parameter>,
    .param_rules        = param_rules,
    .list_options       = list_options,
    .process            = process,
    .start              = start,
    .stop               = stop,
    .set_host_services  = set_host_services,
    .interrupt          = request_stop
)
