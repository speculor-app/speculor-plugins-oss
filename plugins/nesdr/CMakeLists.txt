# NooElec NESDR source plugin. Reuses the rtl_sdr device layer
# (rtl_sdr_device.{h,cpp}, compiled in below) and librtlsdr, which is loaded at
# runtime via LoadLibrary/dlopen — only the rtl-sdr.h header is needed at compile
# time. RtlSdr.cmake provides that header (and, on Windows, the prebuilt
# rtlsdr.dll shared with rtl_sdr); if the fetch fails and no system header is
# present, the plugin self-skips. See ../../THIRD_PARTY_NOTICES.md.
include(RtlSdr)

find_package(RtlSdr QUIET)
if(NOT RtlSdr_FOUND)
    message(STATUS "RTL-SDR header not found — skipping nesdr plugin")
    return()
endif()

spc_add_plugin(nesdr SOURCES
    nesdr_plugin.cpp
    nesdr_models.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../rtl_sdr/rtl_sdr_device.cpp)
target_include_directories(nesdr PRIVATE
    ${RtlSdr_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../rtl_sdr)
