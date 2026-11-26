#pragma once

// country flag image cache for the ADS-B display plugin
// decodes flag PNGs from embedded data (compiled into the DLL)
// draws a text-label fallback for any missing country

#include "draw_helpers.h"
#include "embedded_flags.h"

#include <draw/image.h>
#include <speculor/plugin.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cctype>
#include <string>
#include <unordered_map>

struct CountryFlagCache
{
    static constexpr int FLAG_W = 20;
    static constexpr int FLAG_H = 15;

    // decode all embedded flag PNGs
    // call once during start()
    void load()
    {
        flags_.clear();
        flags_.reserve(embedded_flags::k_flag_count);
        for (size_t i = 0; i < embedded_flags::k_flag_count; ++i) {
            const auto& ef = embedded_flags::k_all_flags[i];
            cv::Mat raw(1, static_cast<int>(ef.size), CV_8UC1,
                        const_cast<uint8_t*>(ef.data));
            auto img = cv::imdecode(raw, cv::IMREAD_COLOR);
            if (img.empty()) continue;
            if (img.cols != FLAG_W || img.rows != FLAG_H)
                cv::resize(img, img, {FLAG_W, FLAG_H}, 0, 0, cv::INTER_AREA);
            spc::draw::Image flag;
            // imdecode returns BGR
            if (flag.assign(img.data, static_cast<uint32_t>(img.cols), static_cast<uint32_t>(img.rows),
                            static_cast<uint32_t>(img.step[0]), SPC_PIXEL_FORMAT_BGR24))
                flags_.emplace(std::string(ef.code), std::move(flag));
        }
    }

    // the FLAG_W x FLAG_H flag for a 2-letter country code (case-insensitive);
    // nullptr when there is none
    const spc::draw::Image* get(const char* code) const
    {
        if (!code || code[0] == '\0' || code[1] == '\0') return nullptr;
        const char lc[3] = {
            static_cast<char>(std::tolower(static_cast<unsigned char>(code[0]))),
            static_cast<char>(std::tolower(static_cast<unsigned char>(code[1]))),
            '\0'
        };
        auto it = flags_.find(lc);
        return it != flags_.end() ? &it->second : nullptr;
    }

    void clear() { flags_.clear(); }
    bool has_flags() const { return !flags_.empty(); }

private:
    std::unordered_map<std::string, spc::draw::Image> flags_;
};

// the flag of `code` with its top-left at (x, y); without one, the code on a plain tile
inline void draw_flag(adsb_draw::DrawList& list, int32_t font, const CountryFlagCache& flags,
                      const char* code, int x, int y)
{
    if (const auto* img = flags.get(code)) {
        adsb_draw::image_at(list, *img, x, y);
        return;
    }
    constexpr int w = CountryFlagCache::FLAG_W, h = CountryFlagCache::FLAG_H;
    adsb_draw::fill_px(list, x, y, x + w - 1, y + h - 1, adsb_draw::rgb({0x3A, 0x47, 0x45}));
    adsb_draw::stroke_px(list, x, y, x + w - 1, y + h - 1, adsb_draw::rgb({0x58, 0x5c, 0x6e}));
    char uc[3] = {'?', '?', '\0'};
    if (code && code[0] != '\0' && code[1] != '\0') {
        uc[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(code[0])));
        uc[1] = static_cast<char>(std::toupper(static_cast<unsigned char>(code[1])));
    }
    const auto st = adsb_draw::style(font, 0.25, {0xcd, 0xd6, 0xf4});
    const float tw = adsb_draw::text_width(uc, st);
    adsb_draw::text_vcenter(list, x + (w - tw) / 2, y, h, uc, st);
}
