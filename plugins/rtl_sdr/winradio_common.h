#pragma once
#include <speculor/plugin_helpers.h>
#include <speculor/table_helpers.h>
#include <speculor/ring_buffer.h>

#include <cstdint>
#include <cstring>
#include <vector>

// ── cross-platform dynamic library loading ──────────────────────────

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
namespace spc::winradio {
using LibHandle = HMODULE;
inline LibHandle lib_open(const char* name) { return LoadLibraryA(name); }
inline void* lib_sym(LibHandle h, const char* name) { return reinterpret_cast<void*>(GetProcAddress(h, name)); }
inline void lib_close(LibHandle h) { if (h) FreeLibrary(h); }
} // namespace spc::winradio
#else
#include <dlfcn.h>
// Windows SDK type compatibility for Linux builds
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif
using INT32 = int32_t;
using UINT32 = uint32_t;
using DWORD_PTR = uintptr_t;
namespace spc::winradio {
using LibHandle = void*;
inline LibHandle lib_open(const char* name) { return dlopen(name, RTLD_NOW); }
inline void* lib_sym(LibHandle h, const char* name) { return dlsym(h, name); }
inline void lib_close(LibHandle h) { if (h) dlclose(h); }
} // namespace spc::winradio
#endif

namespace spc::winradio {

template<typename T>
bool load_fn(LibHandle dll, const char* name, T& out) {
    auto* proc = lib_sym(dll, name);
    if (!proc) return false;
    out = reinterpret_cast<T>(proc);
    return true;
}


// standard I/Q output field indices
enum IqField { F_I = 0, F_Q, IQ_FIELD_COUNT };

// initialize an I/Q output table for int16 signal data
inline void init_iq_table(SpcTable& table, const SpcPortSchema* schema)
{
    uint32_t offsets[IQ_FIELD_COUNT];
    uint32_t stride = 0;
    spc_schema_compute_offsets(schema, offsets, &stride);
    spc_table_init(&table, stride, schema);
}

// populate I/Q table metadata from current SDR state
inline void set_iq_metadata(SpcTable& table,
                            double sample_rate_hz, double center_freq_hz,
                            double bandwidth_hz, double gain_db,
                            bool agc_on, int bit_depth,
                            uint64_t frame_number, int64_t timestamp_us)
{
    table.sample_rate_hz = sample_rate_hz;
    table.center_freq_hz = center_freq_hz;
    table.bandwidth_hz = bandwidth_hz;
    table.gain_db = gain_db;
    table.agc_enabled = agc_on ? 1 : 0;
    table.bit_depth = bit_depth;
    table.frame_number = frame_number;
    table.timestamp_us = timestamp_us;
}

// device registry entry for enum parameter population
struct DeviceEntry {
    int index;  // SDK device index (-1 = None)
    char label[SPC_PARAM_ENUM_LABEL_MAX];
};

// max devices in a registry (limited by SPC_PARAM_ENUM_MAX)
static constexpr int MAX_DEVICES = SPC_PARAM_ENUM_MAX;

// DDC type information (common across WinRadio families)
struct DdcTypeInfo {
    uint32_t sample_rate;      // Hz
    uint32_t bandwidth;        // Hz
    uint32_t bits_per_sample;
};

static constexpr int MAX_DDC_TYPES = SPC_PARAM_ENUM_MAX;

// format a DDC type label showing bandwidth for enum parameter display
inline void format_ddc_label(char* label, size_t label_size, const DdcTypeInfo& info)
{
    if (info.bandwidth >= 1000000)
        std::snprintf(label, label_size, "%.2f MHz",
                      static_cast<double>(info.bandwidth) / 1e6);
    else
        std::snprintf(label, label_size, "%.0f kHz",
                      static_cast<double>(info.bandwidth) / 1e3);
}

} // namespace spc::winradio
