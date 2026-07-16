#include "rtl_sdr_device.h"
#include <speculor/sdr_source_helpers.h>
#include <speculor/sdr_params.h>
#include <spc_clock.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <thread>

// KrakenSDR (KrakenRF) — a 5-channel coherent RTL-SDR array (5x R820T2 +
// RTL2832U sharing one 28.8 MHz TCXO). This is a direct capture driver: it
// opens the five tuners (serials 1000-1004) via the shared RtlSdrDevice layer
// and streams them as five time-aligned I/Q ports.
//
// COHERENCE SCOPE: the shared clock gives FREQUENCY coherence only. The five
// USB streams start at unknown times (a fixed integer-sample offset) and each
// R820T2 PLL locks at a random phase per tune. This driver does NOT do the
// noise-source cross-correlation sample sync + eigen-decomposition phase/amp
// calibration (Heimdall's job) — that is a separate downstream node. Passive
// radar tolerates this: a range-Doppler correlator self-calibrates the range
// origin from the direct-path peak. See README.md.

static constexpr int NUM_CH = 5;

// serial 1000 is the control/reference dongle (owns the noise-source + bias-tee
// GPIO); serials 1001-1004 are the other four channels.
static constexpr int KRAKEN_SERIAL_BASE = 1000;

// ── device detection ────────────────────────────────────────────────

// Locate the five Kraken channels. Primary: match the fixed serial strings
// "1000".."1004" and map channel k -> serial 1000+k (so channel 0 is always the
// control/reference dongle). Fallback (relax): if the serials don't match but
// at least five RTL devices are present, take the first five in enumeration
// order — best-effort for reflashed units, with no guaranteed reference mapping.
static bool detect_kraken(bool relax, uint32_t out_hw_index[NUM_CH], SpcLogContext* log)
{
    if (!spc::rtlsdr::RtlSdrDevice::load_api()) return false;

    auto devs = spc::rtlsdr::RtlSdrDevice::enumerate();
    bool got[NUM_CH] = {};
    int found = 0;
    for (const auto& d : devs) {
        for (int k = 0; k < NUM_CH; ++k) {
            char want[16];
            std::snprintf(want, sizeof(want), "%d", KRAKEN_SERIAL_BASE + k);
            if (!got[k] && std::strcmp(d.serial, want) == 0) {
                out_hw_index[k] = d.index;
                got[k] = true;
                ++found;
            }
        }
    }
    if (found == NUM_CH) return true;

    if (relax && devs.size() >= static_cast<size_t>(NUM_CH)) {
        for (int k = 0; k < NUM_CH; ++k) out_hw_index[k] = devs[k].index;
        if (log) SPC_LOG_WARN(log, "KrakenSDR: serials 1000-1004 not all present; "
                                   "relax_serial_match on — using first %d RTL devices "
                                   "(reference channel not guaranteed)", NUM_CH);
        return true;
    }

    if (log && found > 0)
        SPC_LOG_WARN(log, "KrakenSDR: found %d of %d expected channels (serials 1000-1004)",
                     found, NUM_CH);
    return false;
}

// ── device registry (populated at scan) ─────────────────────────────

static struct KrakenRegistry {
    bool present = false;
    uint32_t hw_index[NUM_CH] = {};

    // gain table queried from channel 0 after open (all five are R820T2, so one
    // table applies to all); used to snap the requested dB gain to a hw step.
    int gains[64] = {};
    int gain_count = 0;

    bool initialized = false;
} g_registry;

// ── state ───────────────────────────────────────────────────────────

struct KrakenSdrState {
    spc::HostServices host;

    std::unique_ptr<spc::rtlsdr::RtlSdrDevice> devices[NUM_CH];

    // parameters — shared across all five tuners unless noted
    int32_t center_freq = 100000000;  // 100 MHz (FM band — a common PR illuminator)
    int32_t sample_rate = 2400000;    // 2.4 MSPS (Kraken recommended max)
    int32_t bandwidth = 0;            // 0 = auto
    int32_t dithering = 0;            // off: the SDM drifts each tuner's phase independently
    int32_t agc_enabled = 0;          // off by default: coherent capture wants manual gain
    float   gain_db = 30.0f;          // universal manual gain, snapped to a hw step
    float   cal_gain_db = -1.0f;      // gain while the noise source is on; -1 = auto (per band)
    int32_t freq_correction = 0;      // PPM (single shared TCXO)

    // Kraken-specific GPIO (all on the channel-0 control dongle)
    int32_t noise_source = 0;         // GPIO 0
    int32_t bias_tee[NUM_CH] = {};    // GPIO 1..5 (per SMA port)

    int32_t relax_serial_match = 0;   // fallback detection for reflashed units

    double actual_sample_rate = 2400000.0;

    // output — one I/Q table per channel port
    SpcTable tables[NUM_CH] = {};
    uint64_t block_index = 0;
    bool streaming = false;

    // overflow reporting (each channel dropping a different amount breaks the
    // fixed inter-channel alignment)
    uint64_t dropped_reported = 0;
    std::chrono::steady_clock::time_point last_drop_log{};
};

SPC_PLUGIN_CAST(KrakenSdrState)
SPC_PLUGIN_HOST_SERVICES(KrakenSdrState, host)

// ── descriptor ──────────────────────────────────────────────────────

static SpcPluginDescriptor g_desc;
static bool g_desc_initialized = false;

static const SpcPluginDescriptor* scan_devices(const SpcHostServices* svc)
{
    SpcLogContext log{};
    if (svc && svc->log) log = {svc->log, svc->host_ctx};

    SPC_LOG_INFO(&log, "KrakenSDR: scanning for device...");
    g_registry.present = detect_kraken(false, g_registry.hw_index, &log);
    g_registry.initialized = true;

    if (g_registry.present)
        SPC_LOG_INFO(&log, "KrakenSDR: detected (channels 0-4 = serials 1000-1004)");
    else
        SPC_LOG_WARN(&log, "KrakenSDR: not detected "
                           "(need 5 RTL-SDR dongles with serials 1000-1004)");

    return &g_desc;
}

static const SpcPluginDescriptor* get_descriptor()
{
    if (!g_desc_initialized) {
        spc::DescriptorBuilder("kraken_sdr", "KrakenSDR", "Signal/SDR/Passive Radar/Sources")
            .author("Speculor").version("0.1.0")
            .data_source()
            .description("Streams five coherent I/Q channels from a KrakenSDR "
                         "(5x R820T2 coherent RTL-SDR array). Frequency-coherent "
                         "capture for passive radar / beamforming — phase calibration "
                         "is a downstream node.")
            .maturity(SPC_MATURITY_PREVIEW)
            .tags({"radio", "sdr", "passive-radar"})
            // five channel outputs; channel 0 = serial 1000 = reference dongle
            .output_signal("iq_ch0", "Channel 0 I/Q (reference)",
                           {{"i", SPC_FIELD_INT16}, {"q", SPC_FIELD_INT16}})
            .output_signal("iq_ch1", "Channel 1 I/Q",
                           {{"i", SPC_FIELD_INT16}, {"q", SPC_FIELD_INT16}})
            .output_signal("iq_ch2", "Channel 2 I/Q",
                           {{"i", SPC_FIELD_INT16}, {"q", SPC_FIELD_INT16}})
            .output_signal("iq_ch3", "Channel 3 I/Q",
                           {{"i", SPC_FIELD_INT16}, {"q", SPC_FIELD_INT16}})
            .output_signal("iq_ch4", "Channel 4 I/Q",
                           {{"i", SPC_FIELD_INT16}, {"q", SPC_FIELD_INT16}})
            // Tuning (shared across all five tuners)
            .int_param(SPC_SDR_CENTER_FREQ, "Center Freq (Hz)",
                       24000000, 1766000000, 100000000, 1000, SPC_SDR_GROUP_TUNING)
                .param_description("Shared receiver center frequency (24 MHz - 1.766 GHz)")
            .int_param(SPC_SDR_SAMPLE_RATE, "Sample Rate (Hz)",
                       900001, 2560000, 2400000, 1000, SPC_SDR_GROUP_TUNING)
                .param_description("Shared ADC sample rate; 2.4 MSPS is the Kraken max")
            .int_param(SPC_SDR_BANDWIDTH, "IF Bandwidth (Hz)",
                       0, 2560000, 0, 10000, SPC_SDR_GROUP_TUNING)
                .param_description("Shared IF bandwidth (0 = automatic, matched to sample rate)")
            // Gain (one universal gain applied identically to all five tuners)
            .bool_param(SPC_SDR_AGC_ENABLED, "AGC", false, SPC_SDR_GROUP_GAIN)
                .param_description("Automatic gain control (leave off for coherent capture)")
            .float_param(SPC_SDR_GAIN, "Manual Gain (dB)",
                         0.0f, 50.0f, 30.0f, 0.1f, SPC_SDR_GROUP_GAIN)
                .param_description("Universal manual gain in dB, snapped to the nearest "
                                   "hardware step and applied to all five tuners (active when AGC is off)")
            .float_param("cal_gain_db", "Calibration Gain (dB)",
                         -1.0f, 50.0f, -1.0f, 0.1f, SPC_SDR_GROUP_GAIN)
                .param_description("Gain applied to all tuners while the noise source is on, "
                                   "restored when it turns off. The burst replaces the antennas, "
                                   "so the operating gain — set for weak off-air signals — rails "
                                   "the 8-bit ADC and degrades the phase estimate. -1 = automatic "
                                   "per-band value (heimdall's calibration gain table)")
            // Hardware / calibration
            .int_param(SPC_SDR_FREQ_CORRECTION, "Freq Correction (PPM)",
                       -100, 100, 0, 1, SPC_SDR_GROUP_HARDWARE)
                .param_description("Clock correction in PPM for the shared TCXO")
            .bool_param(SPC_SDR_DITHERING, "Frequency Dithering", false, SPC_SDR_GROUP_HARDWARE)
                .param_description("Leave OFF for a coherent array: with the R820T2's sigma-delta "
                                   "modulator running, each tuner's phase drifts independently and "
                                   "the array decorrelates itself over minutes, taking any "
                                   "calibration with it. On spreads the fractional-N PLL's spurs "
                                   "into a noise pedestal, which only helps single-channel "
                                   "reception. Exposed because OFF is a claim about this hardware "
                                   "worth being able to test — compare the calibrator's lock "
                                   "quality both ways. R820T-only; a V4's R828D rejects it")
            .bool_param("noise_source", "Calibration Noise Source", false, SPC_SDR_GROUP_HARDWARE)
                .param_description("Toggle the onboard noise source (control dongle GPIO 0): "
                                   "injects a common tone into all channels for a calibration node")
            .bool_param("bias_tee_ch0", "Bias-T Ch0", false, SPC_SDR_GROUP_HARDWARE)
                .param_description("4.5V bias-tee on channel 0 SMA (control dongle GPIO 1) — PREVIEW, verify routing")
            .bool_param("bias_tee_ch1", "Bias-T Ch1", false, SPC_SDR_GROUP_HARDWARE)
                .param_description("4.5V bias-tee on channel 1 SMA (control dongle GPIO 2) — PREVIEW, verify routing")
            .bool_param("bias_tee_ch2", "Bias-T Ch2", false, SPC_SDR_GROUP_HARDWARE)
                .param_description("4.5V bias-tee on channel 2 SMA (control dongle GPIO 3) — PREVIEW, verify routing")
            .bool_param("bias_tee_ch3", "Bias-T Ch3", false, SPC_SDR_GROUP_HARDWARE)
                .param_description("4.5V bias-tee on channel 3 SMA (control dongle GPIO 4) — PREVIEW, verify routing")
            .bool_param("bias_tee_ch4", "Bias-T Ch4", false, SPC_SDR_GROUP_HARDWARE)
                .param_description("4.5V bias-tee on channel 4 SMA (control dongle GPIO 5) — PREVIEW, verify routing")
            .bool_param("relax_serial_match", "Relax Serial Match", false, SPC_SDR_GROUP_HARDWARE)
                .param_description("If serials 1000-1004 aren't found, use the first 5 RTL devices "
                                   "(reflashed units; reference channel not guaranteed)")
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
    auto* s = new KrakenSdrState{};

    auto* desc = get_descriptor();
    for (int k = 0; k < NUM_CH; ++k)
        spc::sdr::init_iq_table(s->tables[k], &desc->ports[k].schema);

    return reinterpret_cast<SpcPluginInstance*>(s);
}

static void close_all(KrakenSdrState* s)
{
    // RtlSdrDevice::close() disables the dongle's bias-tee (SEH-guarded) then
    // cancels + joins the read thread before releasing the handle.
    for (int k = 0; k < NUM_CH; ++k)
        s->devices[k].reset();
}

static void destroy_instance(SpcPluginInstance* inst)
{
    auto* s = state(inst);
    close_all(s);
    for (int k = 0; k < NUM_CH; ++k)
        spc_table_free(&s->tables[k]);
    delete s;
}

// ── parameters ──────────────────────────────────────────────────────

// Snap a requested dB gain to the nearest hardware step.
static int snap_gain_tenths(float gain_db)
{
    int target = static_cast<int>(gain_db * 10.0f + (gain_db >= 0.0f ? 0.5f : -0.5f));
    if (g_registry.gain_count > 0) {
        int best = g_registry.gains[0];
        int best_d = target - best; if (best_d < 0) best_d = -best_d;
        for (int i = 1; i < g_registry.gain_count; ++i) {
            int d = target - g_registry.gains[i]; if (d < 0) d = -d;
            if (d < best_d) { best_d = d; best = g_registry.gains[i]; }
        }
        target = best;
    }
    return target;
}

// Apply the universal manual gain to every open tuner (the Kraken uses one
// "universal gain" across all five).
static void apply_gain_all(KrakenSdrState* s)
{
    const int target = snap_gain_tenths(s->gain_db);
    for (int k = 0; k < NUM_CH; ++k)
        if (s->devices[k]) s->devices[k]->set_tuner_gain(target);
}

// heimdall's per-band calibration gain (hw_controller.py cal_gain_table,
// converted from R820T gain indexes to dB): high enough for the noise source
// to dominate, low enough not to rail the 8-bit ADC.
static float auto_cal_gain_db(int32_t freq_hz)
{
    static constexpr float k_mhz[] = {100, 200, 300, 400, 500, 600, 700, 1700};
    static constexpr float k_db[]  = {8.7f, 16.6f, 22.9f, 25.4f, 33.8f, 40.2f, 49.6f, 49.6f};
    const float f = static_cast<float>(freq_hz) / 1e6f;
    int best = 0;
    float best_d = std::abs(f - k_mhz[0]);
    for (int i = 1; i < 8; ++i) {
        const float d = std::abs(f - k_mhz[i]);
        if (d < best_d) { best_d = d; best = i; }
    }
    return k_db[best];
}

// While the noise source is on, the antennas are switched out — the only
// signal gain has to fit is the burst itself, and at a DAB operating gain it
// rails the 8-bit ADC (measured: 34.8% of burst samples clipped). heimdall
// drops every tuner to a calibration gain for the duration and restores after.
static void enter_cal_gain(KrakenSdrState* s)
{
    const float db = (s->cal_gain_db >= 0.0f) ? s->cal_gain_db
                                              : auto_cal_gain_db(s->center_freq);
    const int target = snap_gain_tenths(db);
    bool all_ok = true;
    for (int k = 0; k < NUM_CH; ++k) {
        if (!s->devices[k]) continue;
        if (s->agc_enabled) s->devices[k]->set_agc(false);   // amplitude cal needs fixed gain
        s->devices[k]->set_tuner_gain_mode(true);
        if (!s->devices[k]->set_tuner_gain(target)) {
            all_ok = false;
            SPC_LOG_ERROR(&s->host.cached_log,
                          "KrakenSDR: ch%d refused calibration gain %.1f dB — its burst may clip", k,
                          static_cast<double>(target) / 10.0);
        }
    }
    if (all_ok)
        SPC_LOG_INFO(&s->host.cached_log,
                     "KrakenSDR: calibration gain %.1f dB engaged for the noise burst "
                     "(operating gain %.1f dB restores when it ends)",
                     static_cast<double>(target) / 10.0, static_cast<double>(s->gain_db));
}

static void leave_cal_gain(KrakenSdrState* s)
{
    for (int k = 0; k < NUM_CH; ++k) {
        if (!s->devices[k]) continue;
        if (s->agc_enabled) {
            s->devices[k]->set_agc(true);
            s->devices[k]->set_tuner_gain_mode(false);
        }
    }
    if (!s->agc_enabled) apply_gain_all(s);
    SPC_LOG_INFO(&s->host.cached_log, "KrakenSDR: operating gain restored (%s)",
                 s->agc_enabled ? "AGC" : "manual");
}

static int set_parameter(SpcPluginInstance* inst, const char* name,
                         const SpcParameterDesc* value)
{
    auto* s = state(inst);
    bool live = s->streaming;
    auto* ctrl = live ? s->devices[0].get() : nullptr;  // GPIO lives on channel 0

    if (spc::try_set_int(name, value, SPC_SDR_CENTER_FREQ, s->center_freq)) {
        if (live) for (int k = 0; k < NUM_CH; ++k)
            if (s->devices[k]) s->devices[k]->set_center_freq(static_cast<uint32_t>(s->center_freq));
        return 0;
    }
    if (spc::try_set_int(name, value, SPC_SDR_SAMPLE_RATE, s->sample_rate)) {
        s->actual_sample_rate = static_cast<double>(s->sample_rate);
        if (live) {
            // librtlsdr soft-resets the demodulator on a rate change, so each
            // channel's stream breaks at a different instant and the array's
            // alignment with it. The calibrator sees the new rate in the
            // metadata and recalibrates.
            for (int k = 0; k < NUM_CH; ++k)
                if (s->devices[k]) s->devices[k]->set_sample_rate(static_cast<uint32_t>(s->sample_rate));
            SPC_LOG_WARN(&s->host.cached_log,
                         "KrakenSDR: sample rate changed while streaming — channel alignment is "
                         "broken until the calibrator recalibrates");
        }
        return 0;
    }
    if (spc::try_set_int(name, value, SPC_SDR_BANDWIDTH, s->bandwidth)) {
        if (live) {
            for (int k = 0; k < NUM_CH; ++k)
                if (s->devices[k]) s->devices[k]->set_bandwidth(static_cast<uint32_t>(s->bandwidth));
            // A bandwidth change re-tunes the PLL under the hood (the filter
            // moves the IF), and each PLL re-locks at a new random phase — but
            // the centre frequency the calibrator watches does not change.
            SPC_LOG_WARN(&s->host.cached_log,
                         "KrakenSDR: bandwidth changed while streaming — each tuner re-locked at a "
                         "new phase; the phase calibration is stale until the next recalibration");
        }
        return 0;
    }
    if (spc::try_set_bool(name, value, SPC_SDR_AGC_ENABLED, s->agc_enabled)) {
        if (live) {
            for (int k = 0; k < NUM_CH; ++k) if (s->devices[k]) {
                s->devices[k]->set_agc(s->agc_enabled != 0);
                s->devices[k]->set_tuner_gain_mode(s->agc_enabled == 0);  // manual when AGC off
            }
            if (!s->agc_enabled) apply_gain_all(s);
        }
        return 0;
    }
    if (spc::try_set_bool(name, value, SPC_SDR_DITHERING, s->dithering)) {
        if (live) for (int k = 0; k < NUM_CH; ++k) if (s->devices[k]) {
            s->devices[k]->set_dithering(s->dithering != 0);
            // Only reaches the PLL on the next programming of it.
            s->devices[k]->set_center_freq(static_cast<uint32_t>(s->center_freq));
        }
        return 0;
    }
    if (spc::try_set_float(name, value, SPC_SDR_GAIN, s->gain_db)) {
        if (live && !s->agc_enabled) apply_gain_all(s);
        return 0;
    }
    if (spc::try_set_int(name, value, SPC_SDR_FREQ_CORRECTION, s->freq_correction)) {
        if (live) for (int k = 0; k < NUM_CH; ++k)
            if (s->devices[k]) s->devices[k]->set_freq_correction(s->freq_correction);
        return 0;
    }
    if (spc::try_set_bool(name, value, "noise_source", s->noise_source)) {
        if (ctrl) {
            // low gain before the switch flips, operating gain only after it
            // flips back — the burst must never meet the operating gain
            if (s->noise_source) enter_cal_gain(s);
            ctrl->set_bias_tee_gpio(0, s->noise_source != 0);
            if (!s->noise_source) leave_cal_gain(s);
        }
        return 0;
    }
    if (spc::try_set_float(name, value, "cal_gain_db", s->cal_gain_db)) return 0;
    for (int k = 0; k < NUM_CH; ++k) {
        char pname[16];
        std::snprintf(pname, sizeof(pname), "bias_tee_ch%d", k);
        if (spc::try_set_bool(name, value, pname, s->bias_tee[k])) {
            if (ctrl) ctrl->set_bias_tee_gpio(k + 1, s->bias_tee[k] != 0);
            return 0;
        }
    }
    if (spc::try_set_bool(name, value, "relax_serial_match", s->relax_serial_match)) return 0;

    return SPC_ERR_NOT_FOUND;
}

static int get_parameter(SpcPluginInstance* inst, const char* name, SpcParameterDesc* out)
{
    auto* s = state(inst);
    bool no_dev = !g_registry.present;
    bool agc_on = (s->agc_enabled != 0);

    if (spc::try_get_int(name, out, SPC_SDR_CENTER_FREQ, s->center_freq)) {
        if (no_dev) out->flags |= SPC_PARAM_FLAG_DISABLED; return 0;
    }
    if (spc::try_get_int(name, out, SPC_SDR_SAMPLE_RATE, s->sample_rate)) {
        if (no_dev) out->flags |= SPC_PARAM_FLAG_DISABLED; return 0;
    }
    if (spc::try_get_int(name, out, SPC_SDR_BANDWIDTH, s->bandwidth)) {
        if (no_dev) out->flags |= SPC_PARAM_FLAG_DISABLED; return 0;
    }
    if (spc::try_get_bool(name, out, SPC_SDR_AGC_ENABLED, s->agc_enabled)) {
        if (no_dev) out->flags |= SPC_PARAM_FLAG_DISABLED; return 0;
    }
    if (spc::try_get_bool(name, out, SPC_SDR_DITHERING, s->dithering)) return 0;
    if (spc::try_get_float(name, out, SPC_SDR_GAIN, s->gain_db)) {
        if (no_dev || agc_on) out->flags |= SPC_PARAM_FLAG_DISABLED; return 0;
    }
    if (spc::try_get_float(name, out, "cal_gain_db", s->cal_gain_db)) {
        if (no_dev) out->flags |= SPC_PARAM_FLAG_DISABLED; return 0;
    }
    if (spc::try_get_int(name, out, SPC_SDR_FREQ_CORRECTION, s->freq_correction)) {
        if (no_dev) out->flags |= SPC_PARAM_FLAG_DISABLED; return 0;
    }
    if (spc::try_get_bool(name, out, "noise_source", s->noise_source)) {
        if (no_dev) out->flags |= SPC_PARAM_FLAG_DISABLED; return 0;
    }
    for (int k = 0; k < NUM_CH; ++k) {
        char pname[16];
        std::snprintf(pname, sizeof(pname), "bias_tee_ch%d", k);
        if (spc::try_get_bool(name, out, pname, s->bias_tee[k])) {
            if (no_dev) out->flags |= SPC_PARAM_FLAG_DISABLED; return 0;
        }
    }
    if (spc::try_get_bool(name, out, "relax_serial_match", s->relax_serial_match)) return 0;

    return SPC_ERR_NOT_FOUND;
}

// ── streaming ───────────────────────────────────────────────────────

static int start(SpcPluginInstance* inst)
{
    auto* s = state(inst);
    s->block_index = 0;
    s->dropped_reported = 0;
    s->last_drop_log = {};

    if (!spc::rtlsdr::RtlSdrDevice::load_api()) {
        SPC_LOG_ERROR(&s->host.cached_log, "KrakenSDR: rtlsdr library not available");
        return 0;  // graceful no-op — pipeline runs, this source produces nothing
    }

    uint32_t idx[NUM_CH];
    bool present = g_registry.present;
    if (present) std::memcpy(idx, g_registry.hw_index, sizeof(idx));
    else present = detect_kraken(s->relax_serial_match != 0, idx, &s->host.cached_log);

    if (!present) {
        SPC_LOG_WARN(&s->host.cached_log,
                     "KrakenSDR: no device — need 5 RTL-SDR dongles (serials 1000-1004)");
        return 0;  // graceful no-op
    }

    // open all five tuners; coherence requires the whole array, so a partial
    // open is a hard failure
    for (int k = 0; k < NUM_CH; ++k) {
        s->devices[k] = std::make_unique<spc::rtlsdr::RtlSdrDevice>(&s->host.cached_log);
        if (!s->devices[k]->open(idx[k])) {
            SPC_LOG_ERROR(&s->host.cached_log, "KrakenSDR: failed to open channel %d", k);
            close_all(s);
            return -1;
        }
    }

    // gain steps are identical across the five R820T2s — query channel 0 once
    g_registry.gain_count = s->devices[0]->query_tuner_gains(g_registry.gains, 64);

    // configure all five identically
    uint32_t rate = static_cast<uint32_t>(s->sample_rate);
    s->actual_sample_rate = static_cast<double>(rate);
    for (int k = 0; k < NUM_CH; ++k) {
        auto* d = s->devices[k].get();
        // A tuner that refuses a setting stays where it was, and a channel parked
        // on the wrong frequency is indistinguishable from one whose antenna is
        // dead: both are flat noise with a DC spike at centre.
        if (!d->set_sample_rate(rate))
            SPC_LOG_ERROR(&s->host.cached_log, "KrakenSDR: ch%d rejected sample rate %u Hz", k, rate);
        // Before the frequency, not after: the setting takes effect at the next
        // PLL programming, so disabling it once the tuner has already been tuned
        // leaves the sigma-delta modulator running until something retunes.
        //
        // With the SDM on, each tuner's phase drifts slowly and independently —
        // the array decorrelates itself over minutes, and any calibration goes
        // stale behind it. Not every librtlsdr exports this.
        if (!d->set_dithering(s->dithering != 0) && s->dithering == 0)
            SPC_LOG_WARN(&s->host.cached_log, "KrakenSDR: ch%d cannot disable dithering — %s, "
                         "so the tuner's sigma-delta modulator stays on and this channel's phase "
                         "will drift. Raise recal_interval_s on the calibrator to chase it", k,
                         d->is_v4() ? "the R828D tuner does not support the call"
                                    : "this librtlsdr has no rtlsdr_set_dithering");
        if (!d->set_center_freq(static_cast<uint32_t>(s->center_freq)))
            SPC_LOG_ERROR(&s->host.cached_log, "KrakenSDR: ch%d rejected center freq %d Hz — this "
                          "channel is NOT tuned where the others are", k, s->center_freq);
        d->set_bandwidth(static_cast<uint32_t>(s->bandwidth));   // 0 = auto; absent in some forks
        if (!d->set_freq_correction(s->freq_correction) && s->freq_correction != 0)
            SPC_LOG_WARN(&s->host.cached_log, "KrakenSDR: ch%d rejected %d ppm correction",
                         k, s->freq_correction);
        if (s->agc_enabled) {
            d->set_agc(true);
            d->set_tuner_gain_mode(false);
        } else {
            d->set_agc(false);
            if (!d->set_tuner_gain_mode(true))
                SPC_LOG_ERROR(&s->host.cached_log, "KrakenSDR: ch%d refused manual gain mode", k);
        }
    }
    if (!s->agc_enabled) apply_gain_all(s);

    // Ask each tuner what it actually did. Without this, a channel that never
    // took the frequency looks exactly like one with a dead antenna.
    for (int k = 0; k < NUM_CH; ++k) {
        const uint32_t f = s->devices[k]->get_center_freq();
        const uint32_t r = s->devices[k]->get_sample_rate();
        if (f == 0 && r == 0) continue;   // read-back unavailable in this build
        const auto want_f = static_cast<uint32_t>(s->center_freq);
        // librtlsdr reports the requested frequency back (and zeroes it when a
        // tune fails), not the PLL's synthesized value — the actual grid snap
        // (up to ~±220 Hz, common to all five) is invisible here. So any real
        // mismatch means the set failed; the tolerance only absorbs fork quirks.
        const bool freq_bad = (f > want_f ? f - want_f : want_f - f) > 100000u;
        if (freq_bad || r != rate)
            SPC_LOG_ERROR(&s->host.cached_log,
                          "KrakenSDR: ch%d reads back %u Hz @ %u Hz, was set to %u Hz @ %u Hz",
                          k, f, r, want_f, rate);
        else
            SPC_LOG_INFO(&s->host.cached_log, "KrakenSDR: ch%d tuned %u Hz @ %u Hz", k, f, r);
    }

    // GPIO (noise source + per-channel bias tees) all live on the channel-0
    // control dongle: GPIO 0 = noise source, GPIO 1..5 = bias tees
    if (s->noise_source) enter_cal_gain(s);   // saved-on source must not meet operating gain
    s->devices[0]->set_bias_tee_gpio(0, s->noise_source != 0);
    for (int k = 0; k < NUM_CH; ++k)
        s->devices[0]->set_bias_tee_gpio(k + 1, s->bias_tee[k] != 0);

    // start the five async reads back-to-back — as near-simultaneous as we can
    // get without a hardware barrier; the residual integer-sample offset is
    // fixed and self-recovered downstream
    for (int k = 0; k < NUM_CH; ++k) {
        if (!s->devices[k]->start_streaming()) {
            SPC_LOG_ERROR(&s->host.cached_log, "KrakenSDR: failed to start channel %d", k);
            close_all(s);
            return -1;
        }
    }

    s->streaming = true;
    SPC_LOG_INFO(&s->host.cached_log,
                 "KrakenSDR started (%d ch, freq=%d Hz, rate=%u Hz, noise_src=%d)",
                 NUM_CH, s->center_freq, rate, s->noise_source);
    return 0;
}

static int stop(SpcPluginInstance* inst)
{
    auto* s = state(inst);
    s->streaming = false;
    close_all(s);
    SPC_LOG_INFO(&s->host.cached_log, "KrakenSDR stopped (%llu blocks)",
                 static_cast<unsigned long long>(s->block_index));
    return 0;
}

// ── process ─────────────────────────────────────────────────────────

// One direct-call block per channel per tick. 65536 samples ≈ 27 ms @ 2.4 MSPS
// — well under the 2 M-sample ring, and few enough calls to keep per-block
// overhead low across the five ports.
static constexpr uint32_t BATCH_SIZE = 65536;

static int process(SpcPluginInstance* inst, const SpcData*, uint32_t,
                   SpcData* outputs, uint32_t output_count)
{
    auto* s = state(inst);
    if (output_count < static_cast<uint32_t>(NUM_CH)) return -1;

    if (!s->streaming) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return 0;
    }

    // A full ring drops samples, and each channel drops a different amount —
    // which silently breaks the fixed inter-channel alignment everything
    // coherent downstream depends on. Say so; the calibrator's next
    // recalibration re-measures and re-aligns.
    uint64_t dropped = 0;
    for (int k = 0; k < NUM_CH; ++k)
        if (s->devices[k]) dropped += s->devices[k]->dropped_samples();
    if (dropped != s->dropped_reported) {
        const auto now_t = std::chrono::steady_clock::now();
        if (now_t - s->last_drop_log > std::chrono::seconds(5)) {
            s->last_drop_log = now_t;
            SPC_LOG_ERROR(&s->host.cached_log,
                          "KrakenSDR: ring overflow — %llu samples dropped (consumer stalled). "
                          "Channel alignment is broken until the calibrator recalibrates",
                          static_cast<unsigned long long>(dropped - s->dropped_reported));
        }
        s->dropped_reported = dropped;
    }

    // Lockstep gate: only emit when every channel has a full batch buffered, so
    // each block is equal-size and time-corresponding across all five ports.
    // The shared clock keeps the equal-rate rings aligned modulo the fixed
    // startup offset, so this never starves in steady state.
    uint32_t ready = BATCH_SIZE;
    for (int k = 0; k < NUM_CH; ++k)
        ready = std::min(ready, s->devices[k] ? s->devices[k]->available() : 0u);
    if (ready < BATCH_SIZE) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return 0;
    }

    auto ts = spc::clock::now_utc_ns(s->host);

    for (int k = 0; k < NUM_CH; ++k) {
        if (spc_table_resize(&s->tables[k], BATCH_SIZE) != 0) return -1;
        s->devices[k]->read_iq(reinterpret_cast<int16_t*>(s->tables[k].data), BATCH_SIZE);

        // shared metadata + shared frame_number/timestamp so a downstream
        // correlator can pair reference/surveillance samples of the same block
        spc::sdr::set_iq_metadata(
            s->tables[k],
            s->actual_sample_rate,
            static_cast<double>(s->center_freq),
            s->actual_sample_rate,                                   // bandwidth ≈ sample rate
            s->agc_enabled ? 0.0 : static_cast<double>(s->gain_db),
            s->agc_enabled != 0,
            8,                                                       // RTL native bit depth
            s->block_index,
            ts);

        outputs[k].type = SPC_DATA_SIGNAL;
        outputs[k].table = &s->tables[k];
    }

    ++s->block_index;
    return 0;
}

// ── export ──────────────────────────────────────────────────────────

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
    .scan_devices      = scan_devices
)
