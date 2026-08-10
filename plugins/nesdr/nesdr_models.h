#pragma once

// Per-model capability profiles for the NooElec NESDR line.
//
// Every NESDR is an RTL2832U dongle, so librtlsdr drives them all identically —
// but what the board around the chip provides is not identical, and librtlsdr
// cannot report it. Whether a Q-branch is wired for HF, whether a bias tee
// exists and whether software can switch it, and whether offset tuning is real
// are board facts, not driver facts. This table carries them so the plugin can
// disable a control that would otherwise lie about what the hardware does.
//
// Pure data and pure functions — no librtlsdr calls — so the table is testable
// with no radio attached.

#include <cstdint>

namespace spc::nesdr {

// Capability-relevant tuner family. librtlsdr reports RTLSDR_TUNER_R820T for
// both the R820T and the R820T2 (it cannot tell them apart), so there is no
// R820T2 member — the marketing chip name lives in ModelCaps::tuner_name.
enum class Tuner { Unknown, R82XX, E4000 };

enum class BiasTee {
    None,      // no bias-tee circuit on the board
    Gpio,      // software-switchable through the RTL2832U GPIO
    AlwaysOn,  // powered whenever the dongle is (SMArTee): not switchable
};

struct ModelCaps {
    const char* id;          // stable profile id, e.g. "smartee_xtr"
    const char* label;       // display name, e.g. "NESDR SMArTee XTR"
    Tuner       tuner;
    const char* tuner_name;  // marketing chip name for logs/UI

    // Board tuning limits. freq_max_hz holds the manufacturer's figure even
    // when it exceeds what an INT parameter in Hz can carry (see
    // MAX_PARAM_FREQ_HZ) — the table states the hardware, the parameter layer
    // states its own limit.
    uint32_t freq_min_hz, freq_max_hz;
    uint32_t gap_lo_hz, gap_hi_hz;  // 0,0 = contiguous; E4000 L-band gap otherwise

    float gain_min_db, gain_max_db;  // tuner's own gain-step span

    BiasTee bias;
    bool    direct_sampling;  // Q-branch wired, so the control is meaningful
    bool    auto_hf_direct;   // switch to Q-ADC automatically below HF_DIRECT_HZ
    bool    offset_tuning;    // real offset tuning (see note below)
    bool    dithering;        // fractional-N PLL dithering (R82XX only)
    bool    tcxo;             // 0.5 PPM TCXO rather than a plain crystal
};

// Below this, an auto_hf_direct model switches to the Q-ADC.
inline constexpr uint32_t HF_DIRECT_HZ = 24000000;

// SPC_PARAM_INT is int32_t, so a frequency in Hz cannot exceed ~2.147 GHz.
// The E4000's advertised 2.3 GHz ceiling is therefore unreachable through the
// standard center_freq parameter. Widening it would mean leaving the
// "center_freq INT Hz" contract in sdr_params.h, which is what lets a generic
// controller (sdr_control, radio_tuner) drive any SDR source interchangeably —
// not worth trading for the top 150 MHz of one tuner family.
inline constexpr uint32_t MAX_PARAM_FREQ_HZ = 2147000000u;

// Resolve from the USB EEPROM manufacturer/product strings, which is all that
// is available before the device is opened. Null when nothing matches — including
// for a NooElec dongle flashed with stock Realtek strings, which is common on the
// Mini and Nano bodies. Case-insensitive substring matching, most specific first.
const ModelCaps* resolve_by_usb(const char* manufacturer, const char* product);

// Resolve from an opened device's rtlsdr_get_tuner_type(). This is the
// authoritative capability signal — it reads the silicon rather than a string
// anyone can rewrite with rtl_eeprom — but it identifies only the tuner family,
// so it yields a generic profile.
const ModelCaps* resolve_by_tuner(int rtlsdr_tuner_type);

// Table access for the model-override enum. Index 0 of the parameter is
// "Auto-detect"; parameter index i >= 1 maps to model_at(i - 1).
const ModelCaps* model_at(int index);
int              model_count();

// Index of a profile in the table, or -1. Lets the plugin report which entry
// auto-detection landed on through the same enum the user sets by hand.
int index_of(const ModelCaps* caps);

const char* bias_tee_name(BiasTee b);

}  // namespace spc::nesdr
