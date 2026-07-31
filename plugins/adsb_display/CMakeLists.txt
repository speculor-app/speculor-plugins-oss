include(${CMAKE_SOURCE_DIR}/cmake/deps/OpenAIP.cmake)

spc_add_plugin(adsb_display SOURCES
    adsb_display_plugin.cpp
    adsb_display_rendering.cpp
    aircraft_layer.cpp
    airports_layer.cpp
    airspace_layer.cpp
    input_parser.cpp
    interactions.cpp
    map_layers.cpp
    map_tiles.cpp
    openaip_loader.cpp
    view_math.cpp
)
target_link_libraries(adsb_display PRIVATE ${OpenCV_LIBS} SpeculorSDK::spclib_http nlohmann_json::nlohmann_json)
target_include_directories(adsb_display SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})

# Copy the fetched OpenAIP data files next to the plugin DLL so the plugin
# can find them at runtime via its own module path. Without this the paths
# would be baked at build time and the plugin would break whenever it was
# moved or the build cache was wiped.
#
# Layout at runtime:
#   <plugin_dir>/adsb_display.dll
#   <plugin_dir>/data/openaip/airports.json
#   <plugin_dir>/data/openaip/airspace.json
#
# Build directly into the bundle subdir when known so ninja's tracked
# OUTPUT matches the as-shipped layout — otherwise the deploy step would
# move the files post-build and ninja would think they were missing on the
# next invocation, re-running the copy_if_different command.
if(DEFINED SPC_PLUGIN_BUNDLE_adsb_display)
    set(_openaip_deploy_dir
        "${SPC_PLUGIN_OUTPUT_DIR}/${SPC_PLUGIN_BUNDLE_adsb_display}/data/openaip")
else()
    set(_openaip_deploy_dir "${SPC_PLUGIN_OUTPUT_DIR}/data/openaip")
endif()
file(MAKE_DIRECTORY "${_openaip_deploy_dir}")

if(OPENAIP_AIRPORTS_AVAILABLE)
    set(_dep_airports "${_openaip_deploy_dir}/airports.json")
    add_custom_command(
        OUTPUT  "${_dep_airports}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${OPENAIP_AIRPORTS_PATH}" "${_dep_airports}"
        DEPENDS "${OPENAIP_AIRPORTS_PATH}"
        COMMENT "[adsb_display] deploying OpenAIP airports.json next to plugin")
    list(APPEND _openaip_deploy_outputs "${_dep_airports}")
endif()

if(OPENAIP_AIRSPACE_AVAILABLE)
    set(_dep_airspace "${_openaip_deploy_dir}/airspace.json")
    add_custom_command(
        OUTPUT  "${_dep_airspace}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${OPENAIP_AIRSPACE_PATH}" "${_dep_airspace}"
        DEPENDS "${OPENAIP_AIRSPACE_PATH}"
        COMMENT "[adsb_display] deploying OpenAIP airspace.json next to plugin")
    list(APPEND _openaip_deploy_outputs "${_dep_airspace}")
endif()

if(_openaip_deploy_outputs)
    add_custom_target(adsb_display_openaip_data DEPENDS ${_openaip_deploy_outputs})
    add_dependencies(adsb_display adsb_display_openaip_data)
endif()
