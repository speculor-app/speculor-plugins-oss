# cmake/deps/OpenAIP.cmake
#
# Fetches the full OpenAIP airports and airspace datasets for the
# adsb_display plugin's airports and airspace layers.
#
# Requires a free OpenAIP API key — sign up at openaip.net -> account
# settings -> generate a "client ID / API key". Pass it in via either:
#   cmake -DOPENAIP_API_KEY=<key> -B build
# or set the environment variable OPENAIP_API_KEY before configuring.
#
# First configure pages through /api/airports and /api/airspaces in
# blocks of 1000 and writes combined JSON arrays to the build-cache
# directory. Subsequent configures skip the fetch as long as the cache
# files exist (set OPENAIP_FORCE_REFETCH=ON to force a re-download).
#
# Output variables:
#   OPENAIP_DATA_DIR             cache directory
#   OPENAIP_AIRPORTS_PATH        absolute path to airports.json
#   OPENAIP_AIRSPACE_PATH        absolute path to airspace.json
#   OPENAIP_AIRPORTS_AVAILABLE   TRUE / FALSE
#   OPENAIP_AIRSPACE_AVAILABLE   TRUE / FALSE

set(OPENAIP_DATA_DIR "${CMAKE_BINARY_DIR}/_deps/openaip-data"
    CACHE PATH "Directory holding OpenAIP-fetched airspace + airports files")
file(MAKE_DIRECTORY "${OPENAIP_DATA_DIR}")

set(OPENAIP_AIRPORTS_PATH "${OPENAIP_DATA_DIR}/airports.json")
set(OPENAIP_AIRSPACE_PATH "${OPENAIP_DATA_DIR}/airspace.json")

# Resolve API key: explicit -DOPENAIP_API_KEY wins, otherwise env var.
set(_OAIP_KEY "${OPENAIP_API_KEY}")
if(NOT _OAIP_KEY)
    set(_OAIP_KEY "$ENV{OPENAIP_API_KEY}")
endif()

option(OPENAIP_FORCE_REFETCH "Re-download OpenAIP data even if cached" OFF)

# Helper: fetch every page of an OpenAIP endpoint and concatenate the
# `items` arrays into a single JSON array file, then strip the formatting
# whitespace `string(JSON ... GET)` puts back in. That indentation is ~1/3 of
# the ~970 MB payload, and it is the dominant input to both release
# compressors (the -full archive and the adsb bundle), so it gets paid for
# twice on every release. Minifying is safe for every consumer: the loader
# parses with nlohmann::json, which does not care.
function(_openaip_fetch endpoint output_path)
    set(_page 1)
    set(_total_pages 1)
    set(_accum "[")
    set(_first TRUE)
    set(_tmp_page "${OPENAIP_DATA_DIR}/_page.tmp.json")

    while(_page LESS_EQUAL _total_pages)
        file(DOWNLOAD
            "https://api.core.openaip.net/api/${endpoint}?limit=1000&page=${_page}&apiKey=${_OAIP_KEY}"
            "${_tmp_page}"
            STATUS _st
            TIMEOUT 180
            INACTIVITY_TIMEOUT 60)
        list(GET _st 0 _rc)
        if(NOT _rc EQUAL 0)
            list(GET _st 1 _msg)
            message(WARNING "[openaip] ${endpoint} page ${_page} failed (rc=${_rc}: ${_msg})")
            file(REMOVE "${_tmp_page}")
            return()
        endif()

        file(READ "${_tmp_page}" _page_json)

        # Detect auth/other errors — error responses have a top-level "code"
        # or "status" instead of "items".
        string(JSON _has_items ERROR_VARIABLE _err TYPE "${_page_json}" items)
        if(_err OR NOT _has_items STREQUAL "ARRAY")
            # likely an error body — surface it
            message(WARNING "[openaip] ${endpoint} page ${_page} response not an items array: ${_page_json}")
            file(REMOVE "${_tmp_page}")
            return()
        endif()

        string(JSON _total_pages GET "${_page_json}" totalPages)
        string(JSON _items_json GET "${_page_json}" items)

        # strip the brackets from the items array so we can concatenate
        string(REGEX REPLACE "^[ \r\n\t]*\\[" "" _items_json "${_items_json}")
        string(REGEX REPLACE "\\][ \r\n\t]*$" "" _items_json "${_items_json}")
        string(STRIP "${_items_json}" _items_json)

        if(_items_json)
            if(NOT _first)
                set(_accum "${_accum},")
            endif()
            set(_accum "${_accum}${_items_json}")
            set(_first FALSE)
        endif()

        message(STATUS "[openaip] ${endpoint}: page ${_page} / ${_total_pages}")
        math(EXPR _page "${_page} + 1")
    endwhile()

    file(REMOVE "${_tmp_page}")
    set(_accum "${_accum}]")
    file(WRITE "${output_path}" "${_accum}")

    message(STATUS "[openaip] ${endpoint} -> ${output_path}")
endfunction()

# Strip the formatting whitespace out of a fetched/cached data file. Applied to
# the cached path too, not just a fresh fetch — CI re-fetches every release (the
# data dir lives under CMAKE_BINARY_DIR) but a dev tree keeps its copy forever,
# and that copy feeds local bundle builds. The script no-ops in milliseconds on
# a file that is already minified, so running it either way is cheap.
# Python3::Interpreter is already REQUIRED at configure time for
# build_bundles.py --emit-cmake-bundles, so this adds no new dependency.
# Non-fatal: a failure here costs archive size, never correctness.
function(_openaip_minify path)
    if(NOT EXISTS "${path}")
        return()
    endif()
    execute_process(
        COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/../../scripts/minify_json.py"
                "${path}"
        RESULT_VARIABLE _rc
        OUTPUT_VARIABLE _out
        ERROR_VARIABLE  _err)
    if(_rc EQUAL 0)
        string(STRIP "${_out}" _out)
        message(STATUS "[openaip] ${_out}")
    else()
        message(WARNING "[openaip] minify failed (rc=${_rc}): ${_err}")
    endif()
endfunction()

if(NOT _OAIP_KEY)
    message(STATUS "[openaip] OPENAIP_API_KEY not set — airspace + airports layers will have no data. Sign up at openaip.net, then reconfigure with -DOPENAIP_API_KEY=<key> or set the OPENAIP_API_KEY env var.")
else()
    if(OPENAIP_FORCE_REFETCH OR NOT EXISTS "${OPENAIP_AIRPORTS_PATH}")
        message(STATUS "[openaip] fetching airports (this can take a minute on first configure)")
        _openaip_fetch("airports" "${OPENAIP_AIRPORTS_PATH}")
    else()
        message(STATUS "[openaip] airports already cached")
    endif()
    _openaip_minify("${OPENAIP_AIRPORTS_PATH}")

    if(OPENAIP_FORCE_REFETCH OR NOT EXISTS "${OPENAIP_AIRSPACE_PATH}")
        message(STATUS "[openaip] fetching airspaces (this can take a few minutes on first configure)")
        _openaip_fetch("airspaces" "${OPENAIP_AIRSPACE_PATH}")
    else()
        message(STATUS "[openaip] airspace already cached")
    endif()
    _openaip_minify("${OPENAIP_AIRSPACE_PATH}")
endif()

if(EXISTS "${OPENAIP_AIRPORTS_PATH}")
    set(OPENAIP_AIRPORTS_AVAILABLE TRUE)
else()
    set(OPENAIP_AIRPORTS_AVAILABLE FALSE)
endif()

if(EXISTS "${OPENAIP_AIRSPACE_PATH}")
    set(OPENAIP_AIRSPACE_AVAILABLE TRUE)
else()
    set(OPENAIP_AIRSPACE_AVAILABLE FALSE)
endif()
