# cmake/deps/RtlSdr.cmake
# Download RTL-SDR API header from the rtlsdrblog/rtl-sdr-blog fork.
# On Windows, also download the pre-built DLL and copy it to the output directory
# so the runtime library always matches the headers we compile against.

set(_RTLSDR_VERSION "1.3.6")
set(_RTLSDR_INSTALL "${CMAKE_BINARY_DIR}/_deps/rtl-sdr-sdk")
set(_RTLSDR_CHECK "${_RTLSDR_INSTALL}/include/rtl-sdr.h")
set(_RTLSDR_URL "https://github.com/rtlsdrblog/rtl-sdr-blog/archive/refs/tags/v${_RTLSDR_VERSION}.tar.gz")

# ── Step 1: download headers from source ────────────────────────────

if(EXISTS "${_RTLSDR_CHECK}")
    message(STATUS "[spclib-deps] RTL-SDR SDK already downloaded (cached)")
else()
    message(STATUS "[spclib-deps] Downloading RTL-SDR Blog SDK v${_RTLSDR_VERSION} ...")
    set(_RTLSDR_ARCHIVE "${CMAKE_BINARY_DIR}/_deps/rtl-sdr-sdk-archive")

    file(DOWNLOAD "${_RTLSDR_URL}" "${_RTLSDR_ARCHIVE}"
        STATUS _dl_status
        SHOW_PROGRESS
    )
    list(GET _dl_status 0 _dl_rc)
    if(NOT _dl_rc EQUAL 0)
        message(WARNING "[spclib-deps] Failed to download RTL-SDR SDK (rc=${_dl_rc})")
    else()
        file(ARCHIVE_EXTRACT INPUT "${_RTLSDR_ARCHIVE}" DESTINATION "${CMAKE_BINARY_DIR}/_deps")
        file(GLOB _RTLSDR_EXTRACTED "${CMAKE_BINARY_DIR}/_deps/rtl-sdr-blog-${_RTLSDR_VERSION}")
        if(_RTLSDR_EXTRACTED)
            file(REMOVE_RECURSE "${_RTLSDR_INSTALL}")
            file(RENAME "${_RTLSDR_EXTRACTED}" "${_RTLSDR_INSTALL}")
        endif()
        file(REMOVE "${_RTLSDR_ARCHIVE}")
        message(STATUS "[spclib-deps] RTL-SDR SDK extracted to ${_RTLSDR_INSTALL}")
    endif()
endif()

if(EXISTS "${_RTLSDR_CHECK}")
    set(RtlSdr_SDK_DIR "${_RTLSDR_INSTALL}" CACHE PATH "" FORCE)
    unset(RtlSdr_FOUND CACHE)
    unset(RtlSdr_INCLUDE_DIR CACHE)
endif()

# ── Step 2: download pre-built DLL (Windows only) ───────────────────

if(WIN32)
    set(_RTLSDR_DLL "${CMAKE_BINARY_DIR}/bin/rtlsdr.dll")
    if(EXISTS "${_RTLSDR_DLL}")
        message(STATUS "[spclib-deps] RTL-SDR DLL already present (cached)")
    else()
        set(_RTLSDR_DLL_URL "https://github.com/rtlsdrblog/rtl-sdr-blog/releases/latest/download/Release.zip")
        set(_RTLSDR_DLL_ARCHIVE "${CMAKE_BINARY_DIR}/_deps/rtl-sdr-release.zip")

        message(STATUS "[spclib-deps] Downloading RTL-SDR Blog runtime DLL ...")
        file(DOWNLOAD "${_RTLSDR_DLL_URL}" "${_RTLSDR_DLL_ARCHIVE}"
            STATUS _dll_status
            SHOW_PROGRESS
        )
        list(GET _dll_status 0 _dll_rc)
        if(NOT _dll_rc EQUAL 0)
            message(WARNING "[spclib-deps] Failed to download RTL-SDR DLL (rc=${_dll_rc})")
        else()
            set(_RTLSDR_DLL_EXTRACT "${CMAKE_BINARY_DIR}/_deps/rtl-sdr-release")
            file(REMOVE_RECURSE "${_RTLSDR_DLL_EXTRACT}")
            file(ARCHIVE_EXTRACT INPUT "${_RTLSDR_DLL_ARCHIVE}" DESTINATION "${_RTLSDR_DLL_EXTRACT}")

            file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
            file(COPY "${_RTLSDR_DLL_EXTRACT}/x64/rtlsdr.dll" DESTINATION "${CMAKE_BINARY_DIR}/bin")

            file(REMOVE_RECURSE "${_RTLSDR_DLL_EXTRACT}")
            file(REMOVE "${_RTLSDR_DLL_ARCHIVE}")
            message(STATUS "[spclib-deps] RTL-SDR DLL copied to ${CMAKE_BINARY_DIR}/bin/")
        endif()
    endif()
endif()
