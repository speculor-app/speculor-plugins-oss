#include "nesdr_models.h"

#include <rtl-sdr.h>

#include <cctype>
#include <cstdio>
#include <cstring>

namespace spc::nesdr {

namespace {

constexpr float R82XX_GAIN_MIN = 0.0f;   // r82xx_gains[]  in librtlsdr: 0 .. 496 tenths dB
constexpr float R82XX_GAIN_MAX = 49.6f;
constexpr float E4K_GAIN_MIN   = -1.0f;  // e4k_gains[]    in librtlsdr: -10 .. 420 tenths dB
constexpr float E4K_GAIN_MAX   = 42.0f;

constexpr uint32_t E4K_GAP_LO = 1100000000u;
constexpr uint32_t E4K_GAP_HI = 1250000000u;

// Order is the model-override enum's order. The two generic entries come first
// so a user with an unrecognised dongle finds them without scrolling past
// fourteen models they do not own.
const ModelCaps k_models[] = {
    // id             label                        tuner          tuner_name
    //   freq_min     freq_max      gap_lo     gap_hi
    //   gain_min       gain_max     bias              ds     auto   offs   dith   tcxo
    {"generic_r820t", "Generic RTL2832U (R820T2)", Tuner::R82XX, "R820T/R820T2",
       24000000u,   1766000000u,          0u,        0u,
       R82XX_GAIN_MIN, R82XX_GAIN_MAX, BiasTee::Gpio,  true,  false, false, true,  false},

    {"generic_e4000", "Generic RTL2832U (E4000)",  Tuner::E4000, "E4000",
       52000000u,   2200000000u,  E4K_GAP_LO, E4K_GAP_HI,
       E4K_GAIN_MIN,   E4K_GAIN_MAX,   BiasTee::Gpio,  false, false, true,  false, false},

    {"mini",          "NESDR Mini",                Tuner::R82XX, "R820T",
       25000000u,   1750000000u,          0u,        0u,
       R82XX_GAIN_MIN, R82XX_GAIN_MAX, BiasTee::None,  false, false, false, true,  false},

    {"mini2",         "NESDR Mini 2",              Tuner::R82XX, "R820T2",
       25000000u,   1750000000u,          0u,        0u,
       R82XX_GAIN_MIN, R82XX_GAIN_MAX, BiasTee::None,  false, false, false, true,  false},

    {"mini2plus",     "NESDR Mini 2+",             Tuner::R82XX, "R820T2",
       25000000u,   1750000000u,          0u,        0u,
       R82XX_GAIN_MIN, R82XX_GAIN_MAX, BiasTee::None,  false, false, false, true,  true},

    {"nano2",         "NESDR Nano 2",              Tuner::R82XX, "R820T2",
       25000000u,   1750000000u,          0u,        0u,
       R82XX_GAIN_MIN, R82XX_GAIN_MAX, BiasTee::None,  false, false, false, true,  false},

    {"nano2plus",     "NESDR Nano 2+",             Tuner::R82XX, "R820T2",
       25000000u,   1750000000u,          0u,        0u,
       R82XX_GAIN_MIN, R82XX_GAIN_MAX, BiasTee::None,  false, false, false, true,  true},

    {"nano3",         "NESDR Nano 3",              Tuner::R82XX, "R820T2",
       25000000u,   1700000000u,          0u,        0u,
       R82XX_GAIN_MIN, R82XX_GAIN_MAX, BiasTee::None,  false, false, false, true,  true},

    {"smart_v4",      "NESDR SMArt v4",            Tuner::R82XX, "R820T2",
       25000000u,   1750000000u,          0u,        0u,
       R82XX_GAIN_MIN, R82XX_GAIN_MAX, BiasTee::None,  false, false, false, true,  true},

    // The only NESDR NooElec documents as receiving HF: the Q-branch is wired,
    // so direct sampling reaches an antenna rather than a dead pin.
    {"smart_v5",      "NESDR SMArt v5",            Tuner::R82XX, "R820T2/R860",
         100000u,   1750000000u,          0u,        0u,
       R82XX_GAIN_MIN, R82XX_GAIN_MAX, BiasTee::None,  true,  true,  false, true,  true},

    {"smartee_v2",    "NESDR SMArTee v2",          Tuner::R82XX, "R820T2",
       25000000u,   1750000000u,          0u,        0u,
       R82XX_GAIN_MIN, R82XX_GAIN_MAX, BiasTee::AlwaysOn, false, false, false, true, true},

    {"smart_xtr",     "NESDR SMArt XTR",           Tuner::E4000, "E4000",
       55000000u,   2300000000u,  E4K_GAP_LO, E4K_GAP_HI,
       E4K_GAIN_MIN,   E4K_GAIN_MAX,   BiasTee::None,  false, false, true,  false, true},

    {"smartee_xtr",   "NESDR SMArTee XTR",         Tuner::E4000, "E4000",
       55000000u,   2300000000u,  E4K_GAP_LO, E4K_GAP_HI,
       E4K_GAIN_MIN,   E4K_GAIN_MAX,   BiasTee::AlwaysOn, false, false, true, false, true},

    {"xtr",           "NESDR XTR",                 Tuner::E4000, "E4000",
       65000000u,   2300000000u,  E4K_GAP_LO, E4K_GAP_HI,
       E4K_GAIN_MIN,   E4K_GAIN_MAX,   BiasTee::None,  false, false, true,  false, false},

    {"xtr_plus",      "NESDR XTR+",                Tuner::E4000, "E4000",
       65000000u,   2300000000u,  E4K_GAP_LO, E4K_GAP_HI,
       E4K_GAIN_MIN,   E4K_GAIN_MAX,   BiasTee::None,  false, false, true,  false, true},
};

constexpr int k_model_count = static_cast<int>(sizeof(k_models) / sizeof(k_models[0]));

const ModelCaps* by_id(const char* id)
{
    for (const auto& m : k_models)
        if (std::strcmp(m.id, id) == 0) return &m;
    return nullptr;
}

bool contains_ci(const char* haystack, const char* needle)
{
    if (!haystack || !needle || !*needle) return false;
    for (const char* h = haystack; *h; ++h) {
        const char* a = h;
        const char* b = needle;
        while (*a && *b &&
               std::tolower(static_cast<unsigned char>(*a)) ==
               std::tolower(static_cast<unsigned char>(*b))) { ++a; ++b; }
        if (!*b) return true;
    }
    return false;
}

struct Pattern {
    const char* needle;
    const char* id;
};

// Most specific first. "NESDR SMArTee XTR" contains all of "smartee", "smart"
// and "xtr", so the compound names have to be tested before any of their parts
// or the profile silently collapses to the wrong board.
const Pattern k_patterns[] = {
    {"smartee xtr",  "smartee_xtr"},
    {"smartee_xtr",  "smartee_xtr"},
    {"smarteextr",   "smartee_xtr"},
    {"smart xtr",    "smart_xtr"},
    {"smart_xtr",    "smart_xtr"},
    {"smartxtr",     "smart_xtr"},
    {"smartee",      "smartee_v2"},
    {"smart v5",     "smart_v5"},
    {"smart_v5",     "smart_v5"},
    {"smartv5",      "smart_v5"},
    {"smart v4",     "smart_v4"},
    {"smart_v4",     "smart_v4"},
    {"smartv4",      "smart_v4"},
    // A bare "NESDR SMArt" is either a v4 or a v5 and the string cannot say
    // which. Resolving to the v4 is the conservative read: it leaves direct
    // sampling disabled, so a v4 is never told to listen on a Q-branch it does
    // not have. A v5 owner pins the profile with the model parameter.
    {"smart",        "smart_v4"},
    {"xtr+",         "xtr_plus"},
    {"xtr plus",     "xtr_plus"},
    {"xtr_plus",     "xtr_plus"},
    {"nano 3",       "nano3"},
    {"nano3",        "nano3"},
    {"nano 2+",      "nano2plus"},
    {"nano2+",       "nano2plus"},
    {"nano 2 plus",  "nano2plus"},
    {"nano2plus",    "nano2plus"},
    {"nano 2",       "nano2"},
    {"nano2",        "nano2"},
    {"nano",         "nano3"},
    {"mini 2+",      "mini2plus"},
    {"mini2+",       "mini2plus"},
    {"mini 2 plus",  "mini2plus"},
    {"mini2plus",    "mini2plus"},
    {"mini 2",       "mini2"},
    {"mini2",        "mini2"},
    {"mini",         "mini2"},
    // Bare "xtr" last of the XTR spellings so "xtr+" wins when both match.
    {"xtr",          "xtr"},
};

}  // namespace

const ModelCaps* resolve_by_usb(const char* manufacturer, const char* product)
{
    char joined[544];
    std::snprintf(joined, sizeof(joined), "%s %s",
                  manufacturer ? manufacturer : "", product ? product : "");

    for (const auto& p : k_patterns)
        if (contains_ci(joined, p.needle)) return by_id(p.id);

    // Identifiably NooElec but not a model we know: the R820T2 generic is the
    // right floor — it is what all but the XTR bodies carry, and it gates off
    // exactly the controls that lie on a board we cannot identify.
    if (contains_ci(joined, "nooelec") || contains_ci(joined, "nesdr"))
        return by_id("generic_r820t");

    return nullptr;
}

const ModelCaps* resolve_by_tuner(int rtlsdr_tuner_type)
{
    switch (rtlsdr_tuner_type) {
        case RTLSDR_TUNER_E4000:              return by_id("generic_e4000");
        case RTLSDR_TUNER_R820T:
        case RTLSDR_TUNER_R828D:              return by_id("generic_r820t");
        default:                              return nullptr;
    }
}

const ModelCaps* model_at(int index)
{
    if (index < 0 || index >= k_model_count) return nullptr;
    return &k_models[index];
}

int model_count() { return k_model_count; }

int index_of(const ModelCaps* caps)
{
    for (int i = 0; i < k_model_count; ++i)
        if (&k_models[i] == caps) return i;
    return -1;
}

const char* bias_tee_name(BiasTee b)
{
    switch (b) {
        case BiasTee::None:     return "none";
        case BiasTee::Gpio:     return "switchable";
        case BiasTee::AlwaysOn: return "always-on";
    }
    return "unknown";
}

}  // namespace spc::nesdr
