#pragma once

// country flag image cache for the ADS-B display plugin
// decodes flag PNGs from embedded data (compiled into the DLL)
// generates a text-label fallback for any missing country

#include "embedded_flags.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <string>
#include <unordered_map>

struct CountryFlagCache
{
    static constexpr int FLAG_W = 20;
    static constexpr int FLAG_H = 15;

    // decode all embedded flag PNGs into cv::Mat images
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

            // imdecode returns BGR; canvas uses RGB convention
            cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

            if (img.cols != FLAG_W || img.rows != FLAG_H)
                cv::resize(img, img, {FLAG_W, FLAG_H}, 0, 0, cv::INTER_AREA);

            flags_[std::string(ef.code)] = std::move(img);
        }
    }

    // get flag image for a 2-letter country code (case-insensitive)
    // returns a FLAG_W x FLAG_H RGB Mat (from cache or generated fallback)
    const cv::Mat& get(const char* code)
    {
        if (!code || code[0] == '\0')
            return get_unknown();

        char lc[3] = {
            static_cast<char>(std::tolower(static_cast<unsigned char>(code[0]))),
            static_cast<char>(std::tolower(static_cast<unsigned char>(code[1]))),
            '\0'
        };
        std::string key(lc);

        auto it = flags_.find(key);
        if (it != flags_.end())
            return it->second;

        // generate text fallback and cache it
        flags_[key] = make_text_flag(code);
        return flags_[key];
    }

    void clear() { flags_.clear(); }

    bool has_flags() const { return !flags_.empty(); }

private:
    std::unordered_map<std::string, cv::Mat> flags_;

    const cv::Mat& get_unknown()
    {
        static cv::Mat unknown = make_text_flag("??");
        return unknown;
    }

    // generate a small colored rectangle with the 2-letter code
    static cv::Mat make_text_flag(const char* code)
    {
        cv::Mat flag(FLAG_H, FLAG_W, CV_8UC3, cv::Scalar(0x3A, 0x47, 0x45)); // surface1
        cv::rectangle(flag, {0, 0, FLAG_W, FLAG_H}, {0x58, 0x5c, 0x6e}, 1); // surface2 border

        char uc[3] = {
            static_cast<char>(std::toupper(static_cast<unsigned char>(code[0]))),
            static_cast<char>(std::toupper(static_cast<unsigned char>(code[1]))),
            '\0'
        };

        int baseline = 0;
        auto sz = cv::getTextSize(uc, cv::FONT_HERSHEY_SIMPLEX, 0.25, 1, &baseline);
        int tx = (FLAG_W - sz.width) / 2;
        int ty = (FLAG_H + sz.height) / 2;
        cv::putText(flag, uc, {tx, ty}, cv::FONT_HERSHEY_SIMPLEX, 0.25,
                    {0xcd, 0xd6, 0xf4}, 1, cv::LINE_AA);
        return flag;
    }
};
