# RTL-SDR source plugin. librtlsdr (LGPL-2.1+) is loaded at runtime via
# LoadLibrary/dlopen — only the rtl-sdr.h header is needed at compile time.
# RtlSdr.cmake downloads that header (and, on Windows, the prebuilt rtlsdr.dll) at
# configure time; FindRtlSdr then locates it. If the fetch fails / is offline and no
# system header is present, the plugin self-skips. See ../../THIRD_PARTY_NOTICES.md.
include(RtlSdr)

find_package(RtlSdr QUIET)
if(NOT RtlSdr_FOUND)
    message(STATUS "RTL-SDR header not found — skipping rtl_sdr plugin")
    return()
endif()

spc_add_plugin(rtl_sdr SOURCES rtl_sdr_plugin.cpp rtl_sdr_device.cpp)
target_include_directories(rtl_sdr PRIVATE ${RtlSdr_INCLUDE_DIR})
# winradio_common.h is vendored alongside the sources (header-only) and resolved
# same-dir by the #include "winradio_common.h" in the plugin/device files.
