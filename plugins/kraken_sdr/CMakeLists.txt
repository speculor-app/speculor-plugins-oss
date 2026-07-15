# KrakenSDR 5-channel coherent source. Reuses the rtl_sdr device layer
# (rtl_sdr_device.{h,cpp}, compiled in below) and librtlsdr, which is loaded at
# runtime via LoadLibrary/dlopen — only the rtl-sdr.h header is needed at compile
# time. RtlSdr.cmake provides that header (and, on Windows, the prebuilt
# rtlsdr.dll shared with rtl_sdr); if the fetch fails and no system header is
# present, the plugin self-skips. See ../../THIRD_PARTY_NOTICES.md.
include(RtlSdr)

find_package(RtlSdr QUIET)
if(NOT RtlSdr_FOUND)
    message(STATUS "RTL-SDR header not found — skipping kraken_sdr plugin")
    return()
endif()

spc_add_plugin(kraken_sdr SOURCES
    kraken_sdr_plugin.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../rtl_sdr/rtl_sdr_device.cpp)
target_include_directories(kraken_sdr PRIVATE
    ${RtlSdr_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../rtl_sdr)
