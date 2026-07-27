#include "rtl_sdr_device.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#include <excpt.h>  // EXCEPTION_EXECUTE_HANDLER for the bias-T teardown guard
#else
#include <unistd.h>
#include <fcntl.h>
#endif

namespace {
// temporarily suppress stderr output from third-party libraries
// (librtlsdr hardcodes fprintf(stderr) for tuner detection messages)
struct StderrSuppressor {
    int saved_fd = -1;
    StderrSuppressor() {
        std::fflush(stderr);
#ifdef _WIN32
        saved_fd = _dup(_fileno(stderr));
        int null_fd = _open("NUL", _O_WRONLY);
        if (null_fd >= 0) { _dup2(null_fd, _fileno(stderr)); _close(null_fd); }
#else
        saved_fd = dup(fileno(stderr));
        int null_fd = open("/dev/null", O_WRONLY);
        if (null_fd >= 0) { dup2(null_fd, fileno(stderr)); close(null_fd); }
#endif
    }
    ~StderrSuppressor() {
        if (saved_fd < 0) return;
        std::fflush(stderr);
#ifdef _WIN32
        _dup2(saved_fd, _fileno(stderr));
        _close(saved_fd);
#else
        dup2(saved_fd, fileno(stderr));
        close(saved_fd);
#endif
    }
};
} // anonymous namespace

namespace spc::rtlsdr {

static constexpr uint32_t RING_CAPACITY = 2097152; // ~1 second at 2 MHz

RtlSdrApi RtlSdrDevice::api_{};
spc::sdr::LibHandle RtlSdrDevice::dll_handle_ = nullptr;

// ── API loading ─────────────────────────────────────────────────────

bool RtlSdrDevice::load_api(SpcLogContext* log)
{
    if (api_.loaded) return true;

#ifdef _WIN32
    dll_handle_ = spc::sdr::lib_open("rtlsdr.dll");
#else
    dll_handle_ = spc::sdr::lib_open("librtlsdr.so.0");
    if (!dll_handle_)
        dll_handle_ = spc::sdr::lib_open("librtlsdr.so");
#endif
    if (!dll_handle_) {
        // librtlsdr is dlopen'd, not linked, so it is invisible to the
        // packaging: on POSIX it comes from the distro, and nothing in the
        // archive hints at that. Say so once, with the command to fix it —
        // otherwise the only symptom is an empty device list.
        static bool warned = false;
        if (!warned) {
            warned = true;
#ifdef _WIN32
            SPC_LOG_WARN(log, "RTL-SDR: rtlsdr.dll not found. It ships in this "
                              "plugin's vendor/ folder — re-extract the bundle "
                              "if it is missing.");
#else
            SPC_LOG_WARN(log, "RTL-SDR: librtlsdr not found (tried librtlsdr.so.0 "
                              "and librtlsdr.so). Install it, then restart: "
                              "Debian/Ubuntu 'sudo apt install librtlsdr0', "
                              "Fedora 'sudo dnf install rtl-sdr', "
                              "Arch 'sudo pacman -S rtl-sdr'. RTL-SDR and "
                              "KrakenSDR list no devices until then.");
#endif
        }
        return false;
    }

    bool ok = true;
    ok &= spc::sdr::load_fn(dll_handle_, "rtlsdr_get_device_count",       api_.GetDeviceCount);
    ok &= spc::sdr::load_fn(dll_handle_, "rtlsdr_get_device_name",        api_.GetDeviceName);
    ok &= spc::sdr::load_fn(dll_handle_, "rtlsdr_get_device_usb_strings", api_.GetDeviceUsbStrings);
    ok &= spc::sdr::load_fn(dll_handle_, "rtlsdr_open",                   api_.Open);
    ok &= spc::sdr::load_fn(dll_handle_, "rtlsdr_close",                  api_.Close);
    ok &= spc::sdr::load_fn(dll_handle_, "rtlsdr_set_center_freq",        api_.SetCenterFreq);
    ok &= spc::sdr::load_fn(dll_handle_, "rtlsdr_get_center_freq",        api_.GetCenterFreq);
    ok &= spc::sdr::load_fn(dll_handle_, "rtlsdr_set_sample_rate",        api_.SetSampleRate);
    ok &= spc::sdr::load_fn(dll_handle_, "rtlsdr_get_sample_rate",        api_.GetSampleRate);
    ok &= spc::sdr::load_fn(dll_handle_, "rtlsdr_set_tuner_gain_mode",    api_.SetTunerGainMode);
    ok &= spc::sdr::load_fn(dll_handle_, "rtlsdr_set_tuner_gain",         api_.SetTunerGain);
    ok &= spc::sdr::load_fn(dll_handle_, "rtlsdr_get_tuner_gains",        api_.GetTunerGains);
    ok &= spc::sdr::load_fn(dll_handle_, "rtlsdr_set_agc_mode",           api_.SetAgcMode);
    ok &= spc::sdr::load_fn(dll_handle_, "rtlsdr_reset_buffer",           api_.ResetBuffer);
    ok &= spc::sdr::load_fn(dll_handle_, "rtlsdr_read_async",             api_.ReadAsync);
    ok &= spc::sdr::load_fn(dll_handle_, "rtlsdr_cancel_async",           api_.CancelAsync);

    // optional functions (don't fail if missing — older library versions)
    spc::sdr::load_fn(dll_handle_, "rtlsdr_set_tuner_bandwidth",  api_.SetTunerBandwidth);
    spc::sdr::load_fn(dll_handle_, "rtlsdr_set_direct_sampling",  api_.SetDirectSampling);
    spc::sdr::load_fn(dll_handle_, "rtlsdr_set_offset_tuning",    api_.SetOffsetTuning);
    spc::sdr::load_fn(dll_handle_, "rtlsdr_set_bias_tee",         api_.SetBiasTee);
    spc::sdr::load_fn(dll_handle_, "rtlsdr_set_bias_tee_gpio",    api_.SetBiasTeeGpio);
    spc::sdr::load_fn(dll_handle_, "rtlsdr_set_freq_correction",  api_.SetFreqCorrection);
    spc::sdr::load_fn(dll_handle_, "rtlsdr_set_testmode",         api_.SetTestmode);
    spc::sdr::load_fn(dll_handle_, "rtlsdr_set_tuner_if_gain",    api_.SetTunerIfGain);
    spc::sdr::load_fn(dll_handle_, "rtlsdr_get_tuner_type",       api_.GetTunerType);
    spc::sdr::load_fn(dll_handle_, "rtlsdr_set_dithering",       api_.SetDithering);

    if (!ok) {
        spc::sdr::lib_close(dll_handle_);
        dll_handle_ = nullptr;
        api_ = {};
        return false;
    }

    api_.loaded = true;
    return true;
}

// ── constructor / destructor ────────────────────────────────────────

RtlSdrDevice::RtlSdrDevice(SpcLogContext* log) : log_(log)
{
    ring_ = spc_ring_create(RING_CAPACITY, 4); // int16 I/Q pairs
}

RtlSdrDevice::~RtlSdrDevice()
{
    close();
    spc_ring_destroy(ring_);
}

// ── enumeration ─────────────────────────────────────────────────────

std::vector<RtlSdrDeviceInfo> RtlSdrDevice::enumerate()
{
    std::vector<RtlSdrDeviceInfo> result;
    if (!load_api()) return result;

    uint32_t count = api_.GetDeviceCount();
    for (uint32_t i = 0; i < count; ++i) {
        RtlSdrDeviceInfo info{};
        info.index = i;

        const char* name = api_.GetDeviceName(i);
        if (name) std::strncpy(info.name, name, sizeof(info.name) - 1);

        api_.GetDeviceUsbStrings(i, info.manufacturer, info.product, info.serial);
        result.push_back(info);
    }
    return result;
}

// ── lifecycle ───────────────────────────────────────────────────────

bool RtlSdrDevice::open(uint32_t device_index)
{
    if (dev_) close();
    if (!api_.loaded) return false;

    int open_rc;
    { StderrSuppressor quiet; open_rc = api_.Open(&dev_, device_index); }
    if (open_rc != 0) {
        SPC_LOG_ERROR(log_, "Failed to open RTL-SDR device index %u", device_index);
        dev_ = nullptr;
        return false;
    }

    // detect V3 vs V4 from product string
    char product[256]{};
    char serial[256]{};
    char manufacturer[256]{};
    api_.GetDeviceUsbStrings(device_index, manufacturer, product, serial);
    is_v4_ = (std::string_view(product).find("R828D") != std::string_view::npos)
           || (std::string_view(product).find("Blog V4") != std::string_view::npos);

    SPC_LOG_INFO(log_, "Opened RTL-SDR device %u [%s] (%s)", device_index, serial,
                 is_v4_ ? "V4/R828D" : "V3/R820T2");
    return true;
}

// Best-effort "disable bias-T before releasing the device" — folded into
// close() so every teardown path (stop, re-open, destructor) gets it. The
// librtlsdr GPIO write is a USB control transfer that has been seen to fault
// *inside* the driver during teardown under rapid open/close cycling; left
// unguarded (it used to run in the plugin's stop() before close()), that SEH
// is caught by the host and abandons stop(), so Close() never runs and the
// leaked open handle wedges the next rtlsdr_open(). Contain it here in a frame
// with no unwind objects so close() always reaches Close(dev_).
void RtlSdrDevice::disable_bias_tee_safe()
{
    if (!dev_ || !api_.SetBiasTee) return;
#ifdef _WIN32
    __try {
        api_.SetBiasTee(dev_, 0);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        // GPIO write faulted mid-teardown — swallow so Close() still runs
    }
#else
    api_.SetBiasTee(dev_, 0);
#endif
}

void RtlSdrDevice::close()
{
    if (!dev_) return;
    stop_streaming();
    if (faulted_.load(std::memory_order_acquire)) {
        // The dongle faulted mid-stream. Its librtlsdr/libusb transfer state is
        // inconsistent; issuing more USB ops on it (bias-T GPIO write,
        // rtlsdr_close) has been seen to wedge for seconds and to walk libusb's
        // corrupted transfer list. Abandon the handle rather than poke a dead
        // device — a faulted dongle needs a replug/restart to recover anyway,
        // and the leaked handle is released when the process exits.
        SPC_LOG_WARN(log_, "RTL-SDR device faulted — abandoning handle without "
                           "further USB I/O (replug or restart to recover)");
        dev_ = nullptr;
        is_v4_ = false;
        return;
    }
    disable_bias_tee_safe();  // must not skip the Close() below on a fault
    api_.Close(dev_);
    SPC_LOG_INFO(log_, "RTL-SDR device closed");
    dev_ = nullptr;
    is_v4_ = false;
}

// ── tuning ──────────────────────────────────────────────────────────

bool RtlSdrDevice::set_center_freq(uint32_t freq_hz)
{
    return dev_ && api_.SetCenterFreq(dev_, freq_hz) == 0;
}

bool RtlSdrDevice::set_sample_rate(uint32_t rate_hz)
{
    return dev_ && api_.SetSampleRate(dev_, rate_hz) == 0;
}

bool RtlSdrDevice::set_bandwidth(uint32_t bw_hz)
{
    return dev_ && api_.SetTunerBandwidth && api_.SetTunerBandwidth(dev_, bw_hz) == 0;
}

uint32_t RtlSdrDevice::get_center_freq() const
{
    return (dev_ && api_.GetCenterFreq) ? api_.GetCenterFreq(dev_) : 0;
}

uint32_t RtlSdrDevice::get_sample_rate() const
{
    return (dev_ && api_.GetSampleRate) ? api_.GetSampleRate(dev_) : 0;
}

// ── gain ────────────────────────────────────────────────────────────

bool RtlSdrDevice::set_agc(bool enabled)
{
    return dev_ && api_.SetAgcMode(dev_, enabled ? 1 : 0) == 0;
}

bool RtlSdrDevice::set_tuner_gain_mode(bool manual)
{
    return dev_ && api_.SetTunerGainMode(dev_, manual ? 1 : 0) == 0;
}

bool RtlSdrDevice::set_tuner_gain(int gain_tenths_db)
{
    return dev_ && api_.SetTunerGain(dev_, gain_tenths_db) == 0;
}

int RtlSdrDevice::query_tuner_gains(int* gains, int max_count)
{
    if (!dev_) return 0;
    // first call with nullptr returns the count
    int count = api_.GetTunerGains(dev_, nullptr);
    if (count <= 0) return 0;
    if (count > max_count) count = max_count;

    std::vector<int> all_gains(static_cast<size_t>(count));
    api_.GetTunerGains(dev_, all_gains.data());
    std::memcpy(gains, all_gains.data(), static_cast<size_t>(count) * sizeof(int));
    return count;
}

// ── hardware ────────────────────────────────────────────────────────

bool RtlSdrDevice::set_direct_sampling(int mode)
{
    if (!dev_ || !api_.SetDirectSampling) return false;
    StderrSuppressor quiet;
    return api_.SetDirectSampling(dev_, mode) == 0;
}

bool RtlSdrDevice::set_offset_tuning(bool enabled)
{
    return dev_ && api_.SetOffsetTuning && api_.SetOffsetTuning(dev_, enabled ? 1 : 0) == 0;
}

bool RtlSdrDevice::set_bias_tee(bool enabled)
{
    return dev_ && api_.SetBiasTee && api_.SetBiasTee(dev_, enabled ? 1 : 0) == 0;
}

bool RtlSdrDevice::set_bias_tee_gpio(int gpio, bool enabled)
{
    return dev_ && api_.SetBiasTeeGpio && api_.SetBiasTeeGpio(dev_, gpio, enabled ? 1 : 0) == 0;
}

bool RtlSdrDevice::set_freq_correction(int ppm)
{
    if (!dev_ || !api_.SetFreqCorrection) return false;
    // librtlsdr returns -2 when the requested ppm is already set — a no-op,
    // not a failure.
    const int rc = api_.SetFreqCorrection(dev_, ppm);
    return rc == 0 || rc == -2;
}

bool RtlSdrDevice::set_testmode(bool enabled)
{
    return dev_ && api_.SetTestmode && api_.SetTestmode(dev_, enabled ? 1 : 0) == 0;
}

bool RtlSdrDevice::set_tuner_if_gain(int stage, int gain_tenths_db)
{
    return dev_ && api_.SetTunerIfGain && api_.SetTunerIfGain(dev_, stage, gain_tenths_db) == 0;
}

bool RtlSdrDevice::set_dithering(bool enabled)
{
    return dev_ && api_.SetDithering && api_.SetDithering(dev_, enabled ? 1 : 0) == 0;
}

int RtlSdrDevice::get_tuner_type()
{
    if (!dev_ || !api_.GetTunerType) return 0;
    return api_.GetTunerType(dev_);
}

// ── streaming ───────────────────────────────────────────────────────

void RtlSdrDevice::async_callback(unsigned char* buf, uint32_t len, void* ctx)
{
    auto* self = static_cast<RtlSdrDevice*>(ctx);
    if (!self->streaming_.load(std::memory_order_relaxed)) {
        // The librtlsdr fork documents cancel_async as safe only from this
        // callback thread; the cross-thread cancel in stop_streaming() is the
        // common practice but rides on non-atomic flags. Repeating it here
        // guarantees the async loop actually exits.
        self->api_.CancelAsync(self->dev_);
        return;
    }

    // RTL-SDR outputs unsigned 8-bit interleaved I/Q
    // convert to signed int16: (uint8 - 128) * 256
    uint32_t num_samples = len / 2;

    int16_t tmp[16384]; // 8192 I/Q pairs
    uint32_t offset = 0;
    while (offset < num_samples) {
        uint32_t batch = std::min(num_samples - offset, uint32_t{8192});
        for (uint32_t i = 0; i < batch; ++i) {
            uint32_t src = (offset + i) * 2;
            tmp[i * 2]     = static_cast<int16_t>((static_cast<int>(buf[src])     - 128) * 256);
            tmp[i * 2 + 1] = static_cast<int16_t>((static_cast<int>(buf[src + 1]) - 128) * 256);
        }
        const uint32_t wrote = spc_ring_write(self->ring_, tmp, batch);
        if (wrote != batch)
            self->dropped_.fetch_add(batch - wrote, std::memory_order_relaxed);
        offset += batch;
    }
}

void RtlSdrDevice::read_thread_fn()
{
    // rtlsdr_read_async blocks until rtlsdr_cancel_async is called
    // buf_num=0 and buf_len=0 use library defaults
    int ret = api_.ReadAsync(dev_, async_callback, this, 0, 0);
    if (ret != 0 && streaming_.load(std::memory_order_relaxed)) {
        // Error while we still wanted to stream = a real device fault (unplug /
        // I/O error): close() abandons the handle rather than poke a dead
        // dongle. A non-zero return AFTER we flipped streaming_ (our own stop,
        // via the callback-thread cancel) is just librtlsdr's cancel-time
        // NOT_FOUND — the transfers were reaped, the device is fine, and close()
        // releases it normally so it can reopen next start.
        faulted_.store(true, std::memory_order_release);
        SPC_LOG_ERROR(log_, "RTL-SDR async read exited with error %d", ret);
    }
    streaming_.store(false, std::memory_order_release);
    read_exited_.store(true, std::memory_order_release);
}

void RtlSdrDevice::request_stop_streaming()
{
    // Abort-phase interrupt: flip streaming_ so the read thread's own
    // async_callback cancels from inside libusb_handle_events — librtlsdr's
    // documented-safe cancel. A cross-thread rtlsdr_cancel_async races
    // librtlsdr's non-atomic async_status and has been seen to make
    // handle_events return LIBUSB_ERROR_NOT_FOUND, tripping a librtlsdr
    // use-after-free. stop_streaming() joins and has a bounded fallback.
    streaming_.store(false, std::memory_order_release);
}

bool RtlSdrDevice::start_streaming()
{
    if (!dev_ || streaming_) return false;
    spc_ring_reset(ring_);
    dropped_.store(0, std::memory_order_relaxed);

    if (api_.ResetBuffer(dev_) != 0) {
        SPC_LOG_ERROR(log_, "Failed to reset RTL-SDR buffer");
        return false;
    }

    faulted_.store(false, std::memory_order_release);
    read_exited_.store(false, std::memory_order_release);
    streaming_.store(true, std::memory_order_release);
    read_thread_ = std::thread(&RtlSdrDevice::read_thread_fn, this);

    SPC_LOG_INFO(log_, "RTL-SDR streaming started");
    return true;
}

void RtlSdrDevice::stop_streaming()
{
    if (!read_thread_.joinable()) return;
    // Prefer librtlsdr's documented-safe cancel: flip streaming_ so the read
    // thread's own async_callback cancels from inside libusb_handle_events.
    // Give it a bounded window to exit; only if it doesn't (a stalled device
    // with no completing transfer left to run the callback) fall back to the
    // cross-thread rtlsdr_cancel_async, which can race librtlsdr's non-atomic
    // async_status and make handle_events return NOT_FOUND — tripping a
    // librtlsdr use-after-free (see request_stop_streaming).
    streaming_.store(false, std::memory_order_release);
    for (int i = 0; i < 250 && !read_exited_.load(std::memory_order_acquire); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    if (!read_exited_.load(std::memory_order_acquire) && dev_ && api_.CancelAsync)
        api_.CancelAsync(dev_);
    // always join — destroying a joinable std::thread calls std::terminate()
    read_thread_.join();
    SPC_LOG_INFO(log_, "RTL-SDR streaming stopped");
}

uint32_t RtlSdrDevice::read_iq(int16_t* buffer, uint32_t max_samples)
{
    return spc_ring_read(ring_, buffer, max_samples);
}

} // namespace spc::rtlsdr
