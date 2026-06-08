spc_add_plugin(subsense_bgs SOURCES subsense_bgs_plugin.cpp)

# CPU SuBSENSE algorithm (LITIV-derived) is compiled here from common/bgs; the shared
# CoreBgs base + OpenCV come from spclib (linked below). ${SPC_OSS_COMMON_DIR} on the
# include path resolves <bgs/subsense/SuBSENSE.hpp>; <bgs/CoreBgs.hpp> resolves to the SDK.
# (SuBSENSE.cpp is single-threaded — no std::execution::par, so no TBB needed.)
target_sources(subsense_bgs PRIVATE ${SPC_OSS_COMMON_DIR}/bgs/subsense/SuBSENSE.cpp)
target_include_directories(subsense_bgs PRIVATE ${SPC_OSS_COMMON_DIR})
target_link_libraries(subsense_bgs PRIVATE SpeculorSDK::spclib)

# --- optional Vulkan GPU acceleration ---
if(SPC_VULKAN_FOUND AND SPC_GLSLANG_VALIDATOR)
    message(STATUS "SuBSENSE BGS: Vulkan found — building with GPU acceleration")
    spc_enable_gpu(subsense_bgs)
    spc_add_gpu_shaders(subsense_bgs SHADERS subsense_init.comp subsense_process.comp subsense_pack_mask.comp)
    target_sources(subsense_bgs PRIVATE subsense_gpu_pipeline.h subsense_gpu_pipeline.cpp)
else()
    message(STATUS "SuBSENSE BGS: Vulkan not available — building CPU-only")
endif()
