#pragma once
#include <speculor/sdr_source_helpers.h>
#include <speculor/plugin_log.h>
#include <speculor/ring_buffer.h>

#include <rtl-sdr.h>

#include <atomic>
#include <cstdint>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace spc::rtlsdr {

struct RtlSdrDeviceInfo {
    uint32_t index;
    char name[256];
    char serial[256];
    char product[256];
    char manufacturer[256];
};

// runtime-loaded function pointers (signatures from rtl-sdr.h)
struct RtlSdrApi {
    // enumeration
    using fn_get_device_count       = uint32_t (*)();
    using fn_get_device_name        = const char* (*)(uint32_t index);
    using fn_get_device_usb_strings = int (*)(uint32_t index, char* manufact, char* product, char* serial);

    // lifecycle
    using fn_open  = int (*)(rtlsdr_dev_t** dev, uint32_t index);
    using fn_close = int (*)(rtlsdr_dev_t* dev);

    // tuning
    using fn_set_center_freq = int (*)(rtlsdr_dev_t* dev, uint32_t freq);
    using fn_get_center_freq = uint32_t (*)(rtlsdr_dev_t* dev);
    using fn_set_sample_rate = int (*)(rtlsdr_dev_t* dev, uint32_t rate);
    using fn_get_sample_rate = uint32_t (*)(rtlsdr_dev_t* dev);

    // gain
    using fn_set_tuner_gain_mode = int (*)(rtlsdr_dev_t* dev, int manual);
    using fn_set_tuner_gain      = int (*)(rtlsdr_dev_t* dev, int gain);
    using fn_get_tuner_gains     = int (*)(rtlsdr_dev_t* dev, int* gains);
    using fn_set_agc_mode        = int (*)(rtlsdr_dev_t* dev, int on);

    // hardware
    using fn_set_tuner_bandwidth = int (*)(rtlsdr_dev_t* dev, uint32_t bw);
    using fn_set_direct_sampling = int (*)(rtlsdr_dev_t* dev, int on);
    using fn_set_offset_tuning   = int (*)(rtlsdr_dev_t* dev, int on);
    using fn_set_bias_tee        = int (*)(rtlsdr_dev_t* dev, int on);
    using fn_set_bias_tee_gpio   = int (*)(rtlsdr_dev_t* dev, int gpio, int on);
    using fn_set_freq_correction = int (*)(rtlsdr_dev_t* dev, int ppm);
    using fn_set_testmode        = int (*)(rtlsdr_dev_t* dev, int on);
    using fn_set_tuner_if_gain   = int (*)(rtlsdr_dev_t* dev, int stage, int gain);
    using fn_get_tuner_type      = int (*)(rtlsdr_dev_t* dev);
    using fn_set_dithering       = int (*)(rtlsdr_dev_t* dev, int dither);

    // streaming
    using fn_reset_buffer  = int (*)(rtlsdr_dev_t* dev);
    using fn_read_async    = int (*)(rtlsdr_dev_t* dev, rtlsdr_read_async_cb_t cb, void* ctx,
                                     uint32_t buf_num, uint32_t buf_len);
    using fn_cancel_async  = int (*)(rtlsdr_dev_t* dev);

    fn_get_device_count       GetDeviceCount = nullptr;
    fn_get_device_name        GetDeviceName = nullptr;
    fn_get_device_usb_strings GetDeviceUsbStrings = nullptr;
    fn_open                   Open = nullptr;
    fn_close                  Close = nullptr;
    fn_set_center_freq        SetCenterFreq = nullptr;
    fn_get_center_freq        GetCenterFreq = nullptr;
    fn_set_sample_rate        SetSampleRate = nullptr;
    fn_get_sample_rate        GetSampleRate = nullptr;
    fn_set_tuner_gain_mode    SetTunerGainMode = nullptr;
    fn_set_tuner_gain         SetTunerGain = nullptr;
    fn_get_tuner_gains        GetTunerGains = nullptr;
    fn_set_agc_mode           SetAgcMode = nullptr;
    fn_set_tuner_bandwidth    SetTunerBandwidth = nullptr;
    fn_set_direct_sampling    SetDirectSampling = nullptr;
    fn_set_offset_tuning      SetOffsetTuning = nullptr;
    fn_set_bias_tee           SetBiasTee = nullptr;
    fn_set_bias_tee_gpio      SetBiasTeeGpio = nullptr;
    fn_set_freq_correction    SetFreqCorrection = nullptr;
    fn_set_testmode           SetTestmode = nullptr;
    fn_set_tuner_if_gain      SetTunerIfGain = nullptr;
    fn_get_tuner_type         GetTunerType = nullptr;
    fn_set_dithering          SetDithering = nullptr;
    fn_reset_buffer           ResetBuffer = nullptr;
    fn_read_async             ReadAsync = nullptr;
    fn_cancel_async           CancelAsync = nullptr;
    bool loaded = false;
};

class RtlSdrDevice {
public:
    explicit RtlSdrDevice(SpcLogContext* log);
    ~RtlSdrDevice();

    [[nodiscard]] static bool load_api();
    [[nodiscard]] static bool is_api_loaded() { return api_.loaded; }

    // lifecycle
    bool open(uint32_t device_index);
    void close();
    [[nodiscard]] bool is_open() const { return dev_ != nullptr; }

    // tuning
    bool set_center_freq(uint32_t freq_hz);
    bool set_sample_rate(uint32_t rate_hz);
    bool set_bandwidth(uint32_t bw_hz);

    // gain
    bool set_agc(bool enabled);
    bool set_tuner_gain_mode(bool manual);
    bool set_tuner_gain(int gain_tenths_db);
    int  query_tuner_gains(int* gains, int max_count);

    // hardware
    bool set_direct_sampling(int mode); // 0=off, 1=I-ADC, 2=Q-ADC
    bool set_offset_tuning(bool enabled);
    bool set_bias_tee(bool enabled);
    bool set_bias_tee_gpio(int gpio, bool enabled);
    bool set_freq_correction(int ppm);
    bool set_testmode(bool enabled);
    bool set_tuner_if_gain(int stage, int gain_tenths_db);
    bool set_dithering(bool enabled);
    int  get_tuner_type();

    // streaming
    bool start_streaming();
    void stop_streaming();

    // data access (consumer side of ring buffer)
    uint32_t read_iq(int16_t* buffer, uint32_t max_samples);

    // I/Q sample pairs buffered and ready to read (used to align multi-device
    // reads: a coherent multi-channel source gates on all channels having a
    // full batch before reading equal counts from each)
    [[nodiscard]] uint32_t available() const { return ring_ ? spc_ring_available(ring_) : 0; }

    // enumeration
    static std::vector<RtlSdrDeviceInfo> enumerate();

    // V3/V4 detection based on product string
    [[nodiscard]] bool is_v4() const { return is_v4_; }

private:
    // async callback — called on the read_async thread
    static void async_callback(unsigned char* buf, uint32_t len, void* ctx);

    // thread function that runs rtlsdr_read_async (blocks until cancel)
    void read_thread_fn();

    // best-effort bias-T disable during close(), contained against a librtlsdr
    // GPIO-write fault so it can never skip the following device release.
    void disable_bias_tee_safe();

    SpcLogContext* log_;
    rtlsdr_dev_t* dev_ = nullptr;
    SpcRingBuffer* ring_ = nullptr;
    std::atomic<bool> streaming_{false};
    std::thread read_thread_;
    bool is_v4_ = false;

    static RtlSdrApi api_;
    static spc::sdr::LibHandle dll_handle_;
};

} // namespace spc::rtlsdr
