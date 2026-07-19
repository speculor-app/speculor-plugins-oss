cmake_minimum_required(VERSION 3.24)
project(SpeculorPluginsOpenSource LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── release codegen ────────────────────────────────────────────────────
# These plugins ship in the same bundle tree as speculor-plugins and run in the
# same process, so their codegen has to match it rather than fall back to
# CMake's bare Release defaults. Kept deliberately identical to that repo's
# block; a difference here shows up as an OSS plugin that is quietly slower or
# unsymbolicated next to its neighbours.

option(PORTABLE_BUILD "Build for redistribution (no -march=native)" OFF)
# When PORTABLE_BUILD is ON, the x86-64 ISA baseline to target:
#   v3 = AVX2/BMI2/FMA (~2013 Haswell+, default)  v2 = SSE4.2 (~2009+)  v1 = SSE2
set(SPECULOR_ARCH_BASELINE "v3" CACHE STRING "Portable x86-64 ISA baseline: v3 (AVX2), v2 (SSE4.2), v1 (SSE2)")
set_property(CACHE SPECULOR_ARCH_BASELINE PROPERTY STRINGS v3 v2 v1)
# Opt-in PDB emission in Release for crash_guard stack-trace symbolication. Off
# by default because PDBs roughly double package size; the app sets this per
# build via the build script.
option(SPECULOR_WITH_PDB "Emit PDBs in Release for crash symbolication" OFF)
# Release-only gate — mirrors speculor-plugins. When ON, only plugins the manifest
# ships are compiled; held-back dirs are skipped. OFF for local dev builds.
option(SPECULOR_RELEASE_PLUGINS "Build only release-shipped plugins (skip in-dev)" OFF)

if(MSVC)
    string(APPEND CMAKE_CXX_FLAGS_RELEASE " /Oi /Ot /fp:fast")
    string(APPEND CMAKE_C_FLAGS_RELEASE   " /Oi /Ot /fp:fast")
    # /arch:AVX2 is x86-only — invalid on ARM64 Windows MSVC (NEON is baseline).
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64"
       AND (NOT PORTABLE_BUILD OR SPECULOR_ARCH_BASELINE STREQUAL "v3"))
        string(APPEND CMAKE_CXX_FLAGS_RELEASE " /arch:AVX2")
        string(APPEND CMAKE_C_FLAGS_RELEASE   " /arch:AVX2")
    endif()
    if(SPECULOR_WITH_PDB)
        # /Zi emits debug info to .pdb; /DEBUG tells the linker to produce one.
        # Re-enable /OPT:REF + /OPT:ICF to counter the size bloat /DEBUG
        # otherwise introduces.
        string(APPEND CMAKE_CXX_FLAGS_RELEASE " /Zi")
        string(APPEND CMAKE_C_FLAGS_RELEASE   " /Zi")
        string(APPEND CMAKE_EXE_LINKER_FLAGS_RELEASE    " /DEBUG /OPT:REF /OPT:ICF")
        string(APPEND CMAKE_SHARED_LINKER_FLAGS_RELEASE " /DEBUG /OPT:REF /OPT:ICF")
        string(APPEND CMAKE_MODULE_LINKER_FLAGS_RELEASE " /DEBUG /OPT:REF /OPT:ICF")
    endif()
else()
    if(PORTABLE_BUILD)
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
            if(SPECULOR_ARCH_BASELINE STREQUAL "v1")
                set(_SPC_ARCH_FLAGS "-march=x86-64")
            elseif(SPECULOR_ARCH_BASELINE STREQUAL "v2")
                set(_SPC_ARCH_FLAGS "-march=x86-64-v2")
            else()
                set(_SPC_ARCH_FLAGS "-march=x86-64-v3")
            endif()
        endif()
    else()
        set(_SPC_ARCH_FLAGS "-march=native -mtune=native")
    endif()
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG ${_SPC_ARCH_FLAGS}")
    set(CMAKE_C_FLAGS_RELEASE   "-O3 -DNDEBUG ${_SPC_ARCH_FLAGS}")
    if(SPECULOR_WITH_PDB)
        # -g1 emits enough DWARF for crash_guard backtraces without inlining
        # details, keeping binary bloat modest.
        string(APPEND CMAKE_CXX_FLAGS_RELEASE " -g1")
        string(APPEND CMAKE_C_FLAGS_RELEASE   " -g1")
    endif()
endif()
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)

# The Speculor SDK bundle is self-contained — it ships its own OpenCV, FFmpeg,
# Vulkan, spclib and the CMake config + helpers. Download a release from
#   https://github.com/speculor-app/speculor-sdk-dist/releases
# extract it, and point CMAKE_PREFIX_PATH at the extracted folder:
#   cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH=/path/to/SpeculorSDK-<ver>-<platform>
find_package(SpeculorSDK REQUIRED)
include(PluginHelpers)      # spc_add_plugin / spc_enable_gpu / spc_add_gpu_shaders
include(CompilerWarnings)   # spc_set_warnings

# local find-modules: RTL-SDR header locator (FindRtlSdr) + downloader (RtlSdr)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/plugins/cmake")

# Built plugins land in build/plugins/ by default; the full build overrides this
# (-DSPC_PLUGIN_OUTPUT_DIR) to route them into the app's shared bin/plugins tree.
if(NOT DEFINED SPC_PLUGIN_OUTPUT_DIR)
    set(SPC_PLUGIN_OUTPUT_DIR ${CMAKE_BINARY_DIR}/plugins)
endif()

# LITIV-derived CPU background-subtraction sources, compiled into the BGS plugins.
# The shared CoreBgs base + OpenCV come from spclib; see common/bgs and each plugin.
set(SPC_OSS_COMMON_DIR ${CMAKE_SOURCE_DIR}/common)

# Plugin → bundle map + in-development gate. spc_add_plugin reads
# SPC_PLUGIN_BUNDLE_<name> to route each plugin DLL into
# <SPC_PLUGIN_OUTPUT_DIR>/<bundle>/. Single source of truth is scripts/bundles.yaml,
# shared with speculor-plugins' build_bundles.py: in the full/release build the
# build script passes -DSPC_PLUGINS_REPO=<speculor-plugins>, so we generate the map
# (and the release gate, and the completeness check) from the manifest. Standalone
# OSS builds (no sibling script) fall back to the static map below and build all.
# Most plugins share the single "oss" bundle; kraken_sdr routes into passive-radar
# — it is the source stage of the coherent chain whose other members
# (coherent_calibrator, range_doppler, pr_map) live in speculor-plugins.
set(_oss_bundles_py "")
if(DEFINED SPC_PLUGINS_REPO AND EXISTS "${SPC_PLUGINS_REPO}/scripts/build_bundles.py")
    set(_oss_bundles_py "${SPC_PLUGINS_REPO}/scripts/build_bundles.py")
endif()
if(_oss_bundles_py)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    set(_oss_bundles_cmake "${CMAKE_BINARY_DIR}/bundles.cmake")
    execute_process(
        COMMAND "${Python3_EXECUTABLE}" "${_oss_bundles_py}"
                --manifest "${CMAKE_SOURCE_DIR}/scripts/bundles.yaml"
                --emit-cmake-bundles "${_oss_bundles_cmake}"
        RESULT_VARIABLE _oss_rc OUTPUT_VARIABLE _oss_out ERROR_VARIABLE _oss_err)
    if(NOT _oss_rc EQUAL 0)
        message(FATAL_ERROR "Failed to emit OSS plugin bundle map (rc=${_oss_rc}):\n${_oss_out}\n${_oss_err}")
    endif()
    include("${_oss_bundles_cmake}")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        "${CMAKE_SOURCE_DIR}/scripts/bundles.yaml")
else()
    # standalone fallback: static map (keep in sync with scripts/bundles.yaml);
    # no in-dev gate — standalone builds compile every plugin.
    foreach(_oss_plugin vibe_bgs subsense_bgs rtl_sdr)
        set(SPC_PLUGIN_BUNDLE_${_oss_plugin} "oss")
    endforeach()
    set(SPC_PLUGIN_BUNDLE_kraken_sdr "passive-radar")
endif()

# Gate add_subdirectory the same way speculor-plugins does (see that repo's
# plugins/CMakeLists.txt): with SPECULOR_RELEASE_PLUGINS ON, a dir the emitted
# fragment marks SPC_PLUGIN_DIR_<token> OFF is held back. No-op when the static
# fallback ran (no SPC_PLUGIN_DIR_* defined) or the option is OFF.
macro(add_subdirectory _spc_dir)
    set(_spc_do_add TRUE)
    if(SPECULOR_RELEASE_PLUGINS)
        file(RELATIVE_PATH _spc_rel "${CMAKE_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/${_spc_dir}")
        string(MAKE_C_IDENTIFIER "${_spc_rel}" _spc_tok)
        if(DEFINED SPC_PLUGIN_DIR_${_spc_tok} AND NOT SPC_PLUGIN_DIR_${_spc_tok})
            set(_spc_do_add FALSE)
            message(STATUS "Speculor release: holding back in-dev plugin dir ${_spc_rel}")
        endif()
    endif()
    if(_spc_do_add)
        _add_subdirectory(${ARGV})
    endif()
endmacro()

add_subdirectory(plugins/vibe_bgs)
add_subdirectory(plugins/subsense_bgs)
add_subdirectory(plugins/rtl_sdr)
add_subdirectory(plugins/kraken_sdr)

# ship the licensing files with any install/redistribution (Apache-2.0 §4 / LGPL)
include(GNUInstallDirs)
install(FILES
    "${CMAKE_SOURCE_DIR}/LICENSE"
    "${CMAKE_SOURCE_DIR}/NOTICE"
    "${CMAKE_SOURCE_DIR}/THIRD_PARTY_NOTICES.md"
    DESTINATION .)
