#pragma once

// Resolve the directory holding the currently-loaded plugin DLL / .so, so
// data files deployed next to the binary can be located at runtime without
// baking build-time absolute paths into the binary. Uses a function pointer
// *inside this translation unit* as the lookup key, which is how both
// `GetModuleHandleEx(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS)` on Windows
// and `dladdr()` on POSIX work.

#include <filesystem>
#include <string>

#ifdef _WIN32
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
    #include <windows.h>
#else
    #include <dlfcn.h>
#endif

namespace spc::plugin_data {

inline std::filesystem::path own_module_dir()
{
#ifdef _WIN32
    HMODULE h = nullptr;
    GetModuleHandleExW(
        GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
        GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
        reinterpret_cast<LPCWSTR>(&own_module_dir),
        &h);
    if (!h) return {};

    wchar_t buf[MAX_PATH];
    DWORD n = GetModuleFileNameW(h, buf, MAX_PATH);
    if (n == 0 || n == MAX_PATH) return {};
    return std::filesystem::path(buf).parent_path();
#else
    Dl_info info{};
    if (dladdr(reinterpret_cast<void*>(&own_module_dir), &info)
        && info.dli_fname) {
        return std::filesystem::path(info.dli_fname).parent_path();
    }
    return {};
#endif
}

} // namespace spc::plugin_data
