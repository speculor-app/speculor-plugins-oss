#pragma once

// Imperial/metric conversions and string formatters for altitude, speed,
// and vertical-rate values. unit_system: 0 = Imperial, 1 = Metric.
// Header-only.

#include <cmath>
#include <cstdint>
#include <format>
#include <string>

inline int32_t ft_to_m(int32_t ft)
{
    return static_cast<int32_t>(std::round(ft * 0.3048));
}

inline float kts_to_kmh(float kts)
{
    return kts * 1.852f;
}

inline int32_t fpm_to_mpm(int32_t fpm)
{
    return static_cast<int32_t>(std::round(fpm * 0.3048));
}

inline std::string format_altitude(int32_t alt_ft, int32_t unit_system)
{
    if (unit_system == 1)
        return std::format("{} m", ft_to_m(alt_ft));
    return std::format("{} ft", alt_ft);
}

inline std::string format_altitude_value(int32_t alt_ft, int32_t unit_system)
{
    if (unit_system == 1)
        return std::format("{}", ft_to_m(alt_ft));
    return std::format("{}", alt_ft);
}

inline std::string format_vspeed(int32_t rate_fpm, int32_t unit_system)
{
    if (unit_system == 1)
        return std::format("{:+d} m/min", fpm_to_mpm(rate_fpm));
    return std::format("{:+d} fpm", rate_fpm);
}

inline std::string format_speed(float gs_kts, int32_t unit_system)
{
    if (unit_system == 1)
        return std::format("{:.0f} km/h", kts_to_kmh(gs_kts));
    return std::format("{:.0f} kts", gs_kts);
}

inline std::string format_speed_value(float gs_kts, int32_t unit_system)
{
    if (unit_system == 1)
        return std::format("{:.0f}", kts_to_kmh(gs_kts));
    return std::format("{:.0f}", gs_kts);
}
