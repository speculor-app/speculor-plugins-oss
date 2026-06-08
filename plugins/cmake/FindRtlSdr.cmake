# FindRtlSdr.cmake — locate the RTL-SDR API header (rtl-sdr.h)
#
# The DLL/SO is loaded at runtime via LoadLibrary/dlopen — only the header
# is needed at compile time.
#
# Sets:
#   RtlSdr_FOUND         - TRUE if header was found
#   RtlSdr_INCLUDE_DIR   - path to the header directory

set(_RTLSDR_SEARCH_PATHS)

if(RtlSdr_SDK_DIR)
    list(APPEND _RTLSDR_SEARCH_PATHS "${RtlSdr_SDK_DIR}")
endif()
if(DEFINED ENV{RTLSDR_SDK_DIR})
    list(APPEND _RTLSDR_SEARCH_PATHS "$ENV{RTLSDR_SDK_DIR}")
endif()

# downloaded cache
list(APPEND _RTLSDR_SEARCH_PATHS "${CMAKE_BINARY_DIR}/_deps/rtl-sdr-sdk")

if(WIN32)
    list(APPEND _RTLSDR_SEARCH_PATHS
        "C:/Program Files/rtl-sdr"
        "C:/Program Files (x86)/rtl-sdr"
    )
else()
    list(APPEND _RTLSDR_SEARCH_PATHS
        "/usr/include"
        "/usr/local/include"
    )
endif()

find_path(RtlSdr_INCLUDE_DIR
    NAMES rtl-sdr.h
    HINTS ${_RTLSDR_SEARCH_PATHS}
    PATH_SUFFIXES include
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(RtlSdr DEFAULT_MSG RtlSdr_INCLUDE_DIR)

if(RtlSdr_FOUND)
    message(STATUS "[RtlSdr] Found header: ${RtlSdr_INCLUDE_DIR}/rtl-sdr.h")
endif()
