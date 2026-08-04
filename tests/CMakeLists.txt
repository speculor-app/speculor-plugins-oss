# One conformance test per plugin, so a failure names the plugin rather than
# reporting that "the plugin tests" broke.
#
# Which plugins exist depends on the release gate in the root CMakeLists, so the
# list is filtered by if(TARGET) instead of being hardcoded — a held-back plugin
# should drop out of the test list, not fail the configure.

set(_candidates
    vibe_bgs
    subsense_bgs
    rtl_sdr
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
