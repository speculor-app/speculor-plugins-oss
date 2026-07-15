# cmake/deps/RtlSdr.cmake
# Download the RTL-SDR API header from the librtlsdr/librtlsdr fork, and on
# Windows the matching pre-built DLL, so the runtime library always matches the
# headers we compile against.
#
# librtlsdr/librtlsdr rather than rtlsdrblog/rtl-sdr-blog because only it exports
# rtlsdr_set_dithering. A coherent array (kraken_sdr) MUST disable the R820T2's
# sigma-delta modulator: with it running, each tuner's phase drifts slowly and
# independently, so the array decorrelates itself over minutes and any
# calibration goes stale behind it. The blog fork has no such call, and its
# absence is silent — the tuners simply keep dithering.
#
# It is a superset of what the blog fork offers (86 exports vs 41), including the
# bias_tee_gpio the KrakenSDR's noise source and bias tees are driven through.

set(_RTLSDR_VERSION "0.9.0")
set(_RTLSDR_INSTALL "${CMAKE_BINARY_DIR}/_deps/rtl-sdr-sdk")
set(_RTLSDR_CHECK "${_RTLSDR_INSTALL}/include/rtl-sdr.h")
set(_RTLSDR_URL "https://github.com/librtlsdr/librtlsdr/archive/refs/tags/v${_RTLSDR_VERSION}.tar.gz")

# ── Step 1: download headers from source ────────────────────────────

if(EXISTS "${_RTLSDR_CHECK}")
    message(STATUS "[spclib-deps] RTL-SDR SDK already downloaded (cached)")
else()
    message(STATUS "[spclib-deps] Downloading librtlsdr SDK v${_RTLSDR_VERSION} ...")
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
        file(GLOB _RTLSDR_EXTRACTED "${CMAKE_BINARY_DIR}/_deps/librtlsdr-${_RTLSDR_VERSION}")
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
        # The static build has libusb linked in, so nothing else has to ship
        # alongside it — matching how the blog fork's Release.zip behaved.
        set(_RTLSDR_DLL_URL "https://github.com/librtlsdr/librtlsdr/releases/download/v${_RTLSDR_VERSION}/rtlsdr-bin-w64_static.zip")
        set(_RTLSDR_DLL_ARCHIVE "${CMAKE_BINARY_DIR}/_deps/rtl-sdr-release.zip")

        message(STATUS "[spclib-deps] Downloading librtlsdr runtime DLL v${_RTLSDR_VERSION} ...")
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
            # Archive is flat and ships it as librtlsdr.dll; the loader asks the
            # OS for "rtlsdr.dll", so land it under that name.
            file(COPY "${_RTLSDR_DLL_EXTRACT}/librtlsdr.dll" DESTINATION "${CMAKE_BINARY_DIR}/bin")
            file(RENAME "${CMAKE_BINARY_DIR}/bin/librtlsdr.dll" "${_RTLSDR_DLL}")

            file(REMOVE_RECURSE "${_RTLSDR_DLL_EXTRACT}")
            file(REMOVE "${_RTLSDR_DLL_ARCHIVE}")
            message(STATUS "[spclib-deps] RTL-SDR DLL copied to ${CMAKE_BINARY_DIR}/bin/")
        endif()
    endif()
endif()
