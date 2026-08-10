// The NESDR plugin's model capability table.
//
// Every NESDR is the same RTL2832U silicon, so what this plugin adds over
// rtl_sdr is entirely the per-model gating: which controls are real on which
// board. That is the part worth asserting, and none of it needs a radio — the
// profile is chosen by the Model parameter, and the contract is visible through
// set_parameter's return code and get_parameter's DISABLED flag.
//
// Three of the four gates exist because the ungated call does something
// actively wrong rather than nothing:
//   - offset_tuning on an R820T/R828D is routed by librtlsdr straight to the
//     bias tee, putting voltage on the antenna port;
//   - direct_sampling on a board with no wired Q-branch tunes the receiver to a
//     pin connected to nothing, and re-initialises the tuner on the way;
//   - bias_tee on a SMArTee cannot switch anything, because the supply is not
//     on that GPIO.
//
// The expectations below deliberately restate the table rather than reading it
// from the plugin. A test that derived them from the same source it is checking
// would pass for any table, including a wrong one.

#include <testing/fake_host.h>
#include <testing/plugin_under_test.h>

#include <cstdio>
#include <cstring>
#include <string>

namespace {

int failures = 0;

void check(bool cond, const std::string& what) {
    std::printf("  %-64s %s\n", what.c_str(), cond ? "ok" : "FAIL");
    if (!cond) ++failures;
}

enum Bias { BIAS_NONE, BIAS_GPIO, BIAS_ALWAYS };

struct Expect {
    const char* label;
    bool  direct_sampling;  // Q-branch wired
    bool  offset_tuning;    // real offset tuning (E4000 only)
    bool  dithering;        // R82XX only
    bool  e4000;            // IF gain stages available
    Bias  bias;
};

// Mirrors plugins/nesdr/nesdr_models.cpp. Keep in sync deliberately: a model
// added there and not here fails the "every model has an expectation" check
// below rather than going silently unasserted.
const Expect k_expect[] = {
    {"Generic RTL2832U (R820T2)", true,  false, true,  false, BIAS_GPIO},
    {"Generic RTL2832U (E4000)",  false, true,  false, true,  BIAS_GPIO},
    {"NESDR Mini",                false, false, true,  false, BIAS_NONE},
    {"NESDR Mini 2",              false, false, true,  false, BIAS_NONE},
    {"NESDR Mini 2+",             false, false, true,  false, BIAS_NONE},
    {"NESDR Nano 2",              false, false, true,  false, BIAS_NONE},
    {"NESDR Nano 2+",             false, false, true,  false, BIAS_NONE},
    {"NESDR Nano 3",              false, false, true,  false, BIAS_NONE},
    {"NESDR SMArt v4",            false, false, true,  false, BIAS_NONE},
    {"NESDR SMArt v5",            true,  false, true,  false, BIAS_NONE},
    {"NESDR SMArTee v2",          false, false, true,  false, BIAS_ALWAYS},
    {"NESDR SMArt XTR",           false, true,  false, true,  BIAS_NONE},
    {"NESDR SMArTee XTR",         false, true,  false, true,  BIAS_ALWAYS},
    {"NESDR XTR",                 false, true,  false, true,  BIAS_NONE},
    {"NESDR XTR+",                false, true,  false, true,  BIAS_NONE},
};
constexpr int k_expect_count = static_cast<int>(sizeof(k_expect) / sizeof(k_expect[0]));

const SpcParameterDesc* find_param(const SpcPluginDescriptor* d, const char* name) {
    if (!d) return nullptr;
    for (uint32_t i = 0; i < d->param_count && i < SPC_PLUGIN_PARAM_MAX; ++i)
        if (std::strcmp(d->params[i].name, name) == 0) return &d->params[i];
    return nullptr;
}

const Expect* expect_for(const char* label) {
    for (const auto& e : k_expect)
        if (std::strcmp(e.label, label) == 0) return &e;
    return nullptr;
}

SpcParameterDesc make_bool(int v) {
    SpcParameterDesc p{};
    p.type = SPC_PARAM_BOOL;
    p.bool_val.value = v;
    return p;
}

SpcParameterDesc make_enum(int v) {
    SpcParameterDesc p{};
    p.type = SPC_PARAM_ENUM;
    p.enum_val.value = v;
    p.enum_val.count = 8;
    return p;
}

SpcParameterDesc make_int(int v) {
    SpcParameterDesc p{};
    p.type = SPC_PARAM_INT;
    p.int_val.value = v;
    return p;
}

bool read_scalar(spc::testing::PluginUnderTest& plugin, const char* name, int& out) {
    SpcParameterDesc d{};
    if (plugin.get_parameter(name, &d) != 0) return false;
    switch (d.type) {
        case SPC_PARAM_BOOL: out = d.bool_val.value; return true;
        case SPC_PARAM_ENUM: out = d.enum_val.value; return true;
        case SPC_PARAM_INT:  out = d.int_val.value;  return true;
        default: return false;
    }
}

// A gated control must answer SPC_ERR_NOT_FOUND when the board does not have
// it, and 0 when it does — and a rejected set must leave no trace. Reporting
// "unsupported" while quietly keeping the value is the worse failure of the
// two: it hands the next start() a setting the user was told was refused.
void check_gate(spc::testing::PluginUnderTest& plugin, const char* label,
                const char* param, const SpcParameterDesc& v, int requested,
                bool supported) {
    int before = 0;
    const bool readable = read_scalar(plugin, param, before);

    const int rc = plugin.set_parameter(param, &v);
    const bool ok = supported ? (rc == 0) : (rc == SPC_ERR_NOT_FOUND);
    check(ok, std::string(label) + ": " + param +
              (supported ? " accepted" : " rejected as unsupported"));
    if (!ok) std::printf("    set_parameter(\"%s\") returned %d\n", param, rc);

    int after = 0;
    if (!readable || !read_scalar(plugin, param, after)) {
        check(false, std::string(label) + ": " + param + " is readable");
        return;
    }
    if (supported) {
        check(after == requested,
              std::string(label) + ": " + param + " stored the requested value");
    } else {
        check(after == before,
              std::string(label) + ": " + param + " rejected set left the value unchanged");
        if (after != before)
            std::printf("    %s changed %d -> %d despite being rejected\n", param, before, after);
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <path-to-nesdr-plugin>\n", argv[0]);
        return 2;
    }

    spc::testing::PluginUnderTest plugin;
    if (!plugin.load(argv[1])) {
        std::fprintf(stderr, "FAIL: %s\n", plugin.error().c_str());
        return 1;
    }
    const auto* d = plugin.descriptor();
    std::printf("plugin: %s\n", d && d->id[0] ? d->id : "<no id>");

    // --- the declared frequency range ---------------------------------------
    // The descriptor is static while the profile is not, so the range has to be
    // the union across the line: 100 kHz for the SMArt v5's direct-sampling
    // floor, up to the largest frequency an INT parameter in Hz can carry.
    if (const SpcParameterDesc* f = find_param(d, "center_freq")) {
        check(f->type == SPC_PARAM_INT, "center_freq is an INT parameter (sdr_params.h contract)");
        check(f->int_val.min <= 100000, "center_freq reaches down to the SMArt v5's 100 kHz");
        check(f->int_val.max >= 1750000000, "center_freq reaches the R820T2 ceiling");
        check(f->int_val.max <= 2147483647, "center_freq max is representable as int32");
        std::printf("    center_freq range: %d .. %d Hz\n", f->int_val.min, f->int_val.max);
    } else {
        check(false, "descriptor declares center_freq");
    }

    if (const SpcParameterDesc* g = find_param(d, "mgc_gain")) {
        check(g->float_val.min <= -1.0f, "mgc_gain reaches the E4000's -1.0 dB step");
        check(g->float_val.max >= 49.6f, "mgc_gain reaches the R820T2's 49.6 dB step");
    } else {
        check(false, "descriptor declares mgc_gain");
    }

    // --- the model enum ------------------------------------------------------
    const SpcParameterDesc* model = find_param(d, "model");
    if (!model) {
        check(false, "descriptor declares model");
        std::printf("FAILED: %d\n", ++failures);
        return 1;
    }
    check(model->enum_val.count == k_expect_count + 1,
          "model enum lists Auto-detect plus every profile");
    std::printf("    model enum: %d option(s), expected %d\n",
                model->enum_val.count, k_expect_count + 1);
    check(std::strcmp(model->enum_val.labels[0], "Auto-detect") == 0,
          "model option 0 is Auto-detect");

    // Copy the labels out: get_parameter rewrites the caller's buffer, and the
    // descriptor is shared mutable state that scan_devices also patches.
    std::string labels[SPC_PARAM_ENUM_MAX];
    for (int i = 0; i < model->enum_val.count && i < SPC_PARAM_ENUM_MAX; ++i)
        labels[i] = model->enum_val.labels[i];
    const int model_count = model->enum_val.count;

    check(plugin.create(), "create_instance");
    if (!plugin.instance()) { std::printf("FAILED: %d\n", ++failures); return 1; }
    spc::testing::FakeHost host;
    plugin.set_host_services(host.services());

    // --- per-model capability gating ----------------------------------------
    int matched = 0;
    for (int i = 1; i < model_count; ++i) {
        const std::string& label = labels[i];
        const Expect* e = expect_for(label.c_str());
        if (!e) {
            check(false, "model \"" + label + "\" has an expectation in this test");
            continue;
        }
        ++matched;

        SpcParameterDesc sel = make_enum(i);
        check(plugin.set_parameter("model", &sel) == 0, label + ": selectable");

        const SpcParameterDesc ds  = make_enum(2);   // Q-ADC
        const SpcParameterDesc on  = make_bool(1);
        const SpcParameterDesc off = make_bool(0);
        const SpcParameterDesc stg = make_enum(3);   // stage 4
        const SpcParameterDesc ifg = make_int(20);

        check_gate(plugin, label.c_str(), "direct_sampling", ds,  2,  e->direct_sampling);
        check_gate(plugin, label.c_str(), "offset_tuning",   on,  1,  e->offset_tuning);
        check_gate(plugin, label.c_str(), "dithering",       on,  1,  e->dithering);
        check_gate(plugin, label.c_str(), "if_gain_stage",   stg, 3,  e->e4000);
        check_gate(plugin, label.c_str(), "if_gain",         ifg, 20, e->e4000);
        check_gate(plugin, label.c_str(), "bias_tee",        on,  1,  e->bias != BIAS_NONE);

        // Now ask for the bias tee OFF, which is the request the three board
        // types answer differently: a switchable one obeys, a hardware-permanent
        // one cannot and must still read as on, and a board without the circuit
        // rejects the request outright and leaves the value alone.
        int before = 0;
        const bool had = read_scalar(plugin, "bias_tee", before);
        const int rc = plugin.set_parameter("bias_tee", &off);
        int after = 0;
        if (!had || !read_scalar(plugin, "bias_tee", after)) {
            check(false, label + ": bias_tee is readable");
        } else if (e->bias == BIAS_ALWAYS) {
            check(rc == 0 && after == 1,
                  label + ": bias_tee stays on when asked to switch off (hardware-permanent)");
        } else if (e->bias == BIAS_GPIO) {
            check(rc == 0 && after == 0, label + ": switchable bias_tee obeys off");
        } else {
            check(rc == SPC_ERR_NOT_FOUND && after == before,
                  label + ": bias_tee off is rejected and changes nothing");
        }

        // Leave the switchable controls off, so a value accepted by this model
        // cannot be read as its own by the next one.
        (void)plugin.set_parameter("bias_tee", &off);
        (void)plugin.set_parameter("offset_tuning", &off);
    }

    check(matched == k_expect_count,
          "every expected model appears in the plugin's enum");
    std::printf("    matched %d of %d expected models\n", matched, k_expect_count);

    // --- Auto-detect is still selectable ------------------------------------
    SpcParameterDesc auto_sel = make_enum(0);
    check(plugin.set_parameter("model", &auto_sel) == 0, "model returns to Auto-detect");

    // --- an unknown parameter is still rejected -----------------------------
    // The gated branches above return SPC_ERR_NOT_FOUND for names that DO exist,
    // so this confirms the fallthrough for names that do not still works.
    const SpcParameterDesc junk = make_bool(1);
    check(plugin.set_parameter("no_such_param", &junk) == SPC_ERR_NOT_FOUND,
          "unknown parameter name rejected");

    plugin.destroy();

    std::printf("%s: %d failure(s)\n", failures ? "FAILED" : "PASSED", failures);
    return failures ? 1 : 0;
}
