cmake_minimum_required(VERSION 3.24)
project(SpeculorPluginsOpenSource LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

# all built plugins land in build/plugins/ — copy these into Speculor's plugins/
set(SPC_PLUGIN_OUTPUT_DIR ${CMAKE_BINARY_DIR}/plugins)

# LITIV-derived CPU background-subtraction sources, compiled into the BGS plugins.
# The shared CoreBgs base + OpenCV come from spclib; see common/bgs and each plugin.
set(SPC_OSS_COMMON_DIR ${CMAKE_SOURCE_DIR}/common)

add_subdirectory(plugins/vibe_bgs)
add_subdirectory(plugins/subsense_bgs)
add_subdirectory(plugins/rtl_sdr)

# ship the licensing files with any install/redistribution (Apache-2.0 §4 / LGPL)
include(GNUInstallDirs)
install(FILES
    "${CMAKE_SOURCE_DIR}/LICENSE"
    "${CMAKE_SOURCE_DIR}/NOTICE"
    "${CMAKE_SOURCE_DIR}/THIRD_PARTY_NOTICES.md"
    DESTINATION .)
