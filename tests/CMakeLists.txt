# One conformance test per plugin, so a failure names the plugin rather than
# reporting that "the plugin tests" broke.
#
# Which plugins exist depends on the release gate in the root CMakeLists, so the
# list is filtered by if(TARGET) instead of being hardcoded — a held-back plugin
# should drop out of the test list, not fail the configure.

if(MSVC)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

set(_candidates
    vibe_bgs
    subsense_bgs
    rtl_sdr
    nesdr
    kraken_sdr
    adsb_display
)

set(_present "")
foreach(_p ${_candidates})
    if(TARGET ${_p})
        list(APPEND _present ${_p})
    endif()
endforeach()

if(NOT _present)
    message(STATUS "[tests] no plugin targets built — skipping conformance tests")
    return()
endif()

spc_add_plugin_test(spc_plugin_conformance
    SOURCES conformance_runner.cpp
    PLUGINS ${_present}
)
# Lends a GPU plugin the machine's device, as the engine does.
spc_enable_gpu(spc_plugin_conformance)

foreach(_p ${_present})
    add_test(NAME conformance.${_p}
             COMMAND spc_plugin_conformance $<TARGET_FILE:${_p}>)
    # The SDK bundle's shared libraries sit beside the plugins rather than on
    # the loader path, and a plugin that cannot resolve them looks identical to
    # one that fails to export a vtable.
    set_tests_properties(conformance.${_p} PROPERTIES TIMEOUT 120)
endforeach()

message(STATUS "[tests] conformance registered for: ${_present}")

# ── behavioural: background subtraction ──────────────────────────────
# Only the BGS plugins share an input/output contract specific enough to assert
# against — image in, foreground mask out. The SDR sources need hardware and
# adsb_display needs a populated track table, so both are covered elsewhere.
set(_bgs "")
foreach(_p vibe_bgs subsense_bgs)
    if(TARGET ${_p})
        list(APPEND _bgs ${_p})
    endif()
endforeach()

if(_bgs)
    spc_add_plugin_test(spc_bgs_behaviour
        SOURCES bgs_behaviour_runner.cpp
        PLUGINS ${_bgs}
    )
    foreach(_p ${_bgs})
        add_test(NAME behaviour.${_p}
                 COMMAND spc_bgs_behaviour $<TARGET_FILE:${_p}>)
        # Slower than conformance: each case settles the model over ~180 frames.
        set_tests_properties(behaviour.${_p} PROPERTIES TIMEOUT 300)
    endforeach()
    message(STATUS "[tests] behaviour registered for: ${_bgs}")
endif()

# ── no-hardware paths: SDR sources ───────────────────────────────────
# CI has no radio, but the paths worth covering do not need one: scanning with
# nothing attached, starting with no device selected, and the shutdown sequence
# — which is where the librtlsdr heap corruption and the unbounded-stop bugs
# both lived.
set(_sdr "")
foreach(_p rtl_sdr nesdr kraken_sdr)
    if(TARGET ${_p})
        list(APPEND _sdr ${_p})
    endif()
endforeach()

if(_sdr)
    spc_add_plugin_test(spc_sdr_nohardware
        SOURCES sdr_nohardware_runner.cpp
        PLUGINS ${_sdr}
    )
    foreach(_p ${_sdr})
        add_test(NAME nohardware.${_p}
                 COMMAND spc_sdr_nohardware $<TARGET_FILE:${_p}>)
        set_tests_properties(nohardware.${_p} PROPERTIES TIMEOUT 120)
    endforeach()
    message(STATUS "[tests] no-hardware registered for: ${_sdr}")
endif()

# ── nesdr: the per-model capability table ────────────────────────────
# What nesdr adds over rtl_sdr is entirely the gating — which controls are real
# on which NooElec board — and none of it needs a radio: the profile is picked
# by the Model parameter, and the contract shows up in a parameter's DISABLED
# flag and in the value that stands after a set. Three of the gates exist
# because the ungated call misbehaves rather than no-ops (offset tuning is
# routed to the bias tee on an R820T2, direct sampling reaches an unwired pin,
# a SMArTee's bias tee is not on the GPIO at all).
if(TARGET nesdr)
    spc_add_plugin_test(spc_nesdr_models
        SOURCES nesdr_models_runner.cpp
        PLUGINS nesdr
    )
    add_test(NAME models.nesdr COMMAND spc_nesdr_models $<TARGET_FILE:nesdr>)
    set_tests_properties(models.nesdr PROPERTIES TIMEOUT 120)
    message(STATUS "[tests] model-capability registered for: nesdr")
endif()

# ── adsb_display: the parts that survive without a display ───────────
# Rendering cannot be asserted headlessly, but the waypoints list is what a
# .speculor file stores and restores — a lossy round-trip loses user data
# silently. The rest is robustness against inputs the GUI really produces.
if(TARGET adsb_display)
    spc_add_plugin_test(spc_adsb_display_tests
        SOURCES adsb_display_runner.cpp
        PLUGINS adsb_display
    )
    add_test(NAME display.adsb_display
             COMMAND spc_adsb_display_tests $<TARGET_FILE:adsb_display>)
    set_tests_properties(display.adsb_display PROPERTIES TIMEOUT 120)
    message(STATUS "[tests] display registered for: adsb_display")
endif()

# ── adsb_display draws the same map on the GPU as on the CPU ──────────
# The same aircraft and receiver position to process() and record_gpu(): the
# frames must agree. SKIPs (exit 0) without a Vulkan device.
if(TARGET adsb_display AND SPC_VULKAN_FOUND)
    spc_add_plugin_test(spc_adsb_display_parity_tests
        SOURCES adsb_display_parity_runner.cpp display_gpu_harness.cpp
        PLUGINS adsb_display
    )
    spc_enable_gpu(spc_adsb_display_parity_tests)
    add_test(NAME display.adsb_display_parity
             COMMAND spc_adsb_display_parity_tests $<TARGET_FILE:adsb_display>)
    set_tests_properties(display.adsb_display_parity PROPERTIES TIMEOUT 180)
    message(STATUS "[tests] display registered for: adsb_display_parity")
endif()

# On Windows a test only finds the libraries the plugins need (the SDK's, the
# app's, the vendor DLLs beside a bundle) when their folders are on PATH: the
# engine adds them with AddDllDirectory, a test calling LoadLibrary cannot, and
# a missing one stalls on a system error dialog until the test times out. The
# same folders speculor-plugins' tests use: the one that holds the plugins
# folder, and each plugin's vendor folder. POSIX finds them through the RPATH
# spc_add_plugin bakes in.
if(WIN32)
    get_filename_component(_spc_runtime_dir "${SPC_PLUGIN_OUTPUT_DIR}" DIRECTORY)
    set(_spc_dll_dirs "${_spc_runtime_dir}")
    foreach(_p ${_present})
        get_target_property(_spc_outdir ${_p} LIBRARY_OUTPUT_DIRECTORY)
        if(_spc_outdir)
            list(APPEND _spc_dll_dirs "${_spc_outdir}/vendor")
        endif()
    endforeach()
    list(REMOVE_DUPLICATES _spc_dll_dirs)
    set(_spc_dll_path_ops "")
    foreach(_d ${_spc_dll_dirs})
        file(TO_NATIVE_PATH "${_d}" _d_native)
        list(APPEND _spc_dll_path_ops "PATH=path_list_prepend:${_d_native}")
    endforeach()
    get_property(_spc_tests DIRECTORY PROPERTY TESTS)
    foreach(_t ${_spc_tests})
        set_tests_properties(${_t} PROPERTIES ENVIRONMENT_MODIFICATION "${_spc_dll_path_ops}")
    endforeach()
endif()
