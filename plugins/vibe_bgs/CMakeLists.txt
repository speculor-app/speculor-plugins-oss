spc_add_plugin(vibe_bgs SOURCES vibe_bgs_plugin.cpp)

# CPU ViBe algorithm (LITIV-derived) is compiled here from common/bgs; the shared
# CoreBgs base + OpenCV come from spclib (linked below). ${SPC_OSS_COMMON_DIR} on the
# include path resolves <bgs/vibe/Vibe.hpp>; <bgs/CoreBgs.hpp> resolves to the SDK.
target_sources(vibe_bgs PRIVATE ${SPC_OSS_COMMON_DIR}/bgs/vibe/Vibe.cpp)
target_include_directories(vibe_bgs PRIVATE ${SPC_OSS_COMMON_DIR})
target_link_libraries(vibe_bgs PRIVATE SpeculorSDK::spclib)

# Vibe.cpp uses std::execution::par; libstdc++ routes that through Intel TBB (spclib
# links it PRIVATE, so it isn't inherited). MSVC's PPL backend needs no extra lib.
if(NOT MSVC)
    find_package(TBB QUIET)
    if(TARGET TBB::tbb)
        target_link_libraries(vibe_bgs PRIVATE TBB::tbb)
    else()
        target_link_libraries(vibe_bgs PRIVATE tbb)
    endif()
endif()

# --- optional Vulkan GPU acceleration ---
if(SPC_VULKAN_FOUND AND SPC_GLSLANG_VALIDATOR)
    message(STATUS "ViBe BGS: Vulkan found — building with GPU acceleration")
    spc_enable_gpu(vibe_bgs)
    spc_add_gpu_shaders(vibe_bgs SHADERS
        vibe_init.comp vibe_process.comp
        vibe_init_wide.comp vibe_process_wide.comp
        vibe_pack_mask.comp)
    target_sources(vibe_bgs PRIVATE vibe_gpu_pipeline.h vibe_gpu_pipeline.cpp)
else()
    message(STATUS "ViBe BGS: Vulkan not available — building CPU-only")
endif()
