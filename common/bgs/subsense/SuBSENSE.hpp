/*
 *   Derived from the LITIV Framework (https://github.com/plstcharles/litiv)
 *   Copyright (c) 2015 Pierre-Luc St-Charles
 *   Modified for Speculor (spclib)
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 */

#pragma once

#include <bgs/CoreBgs.hpp>
#include <bgs/CoreParameters.hpp>
#include "SuBSENSEUtils.hpp"
#include "LBSP.hpp"

#include <opencv2/opencv.hpp>

#include <vector>
#include <random>

namespace spclib::bgs
{
    // SuBSENSE: Self-Balanced Sensitivity Segmenter
    // combines color distance with LBSP (Local Binary Similarity Pattern) texture features.
    // per-pixel adaptive thresholds and learning rates driven by a feedback loop.
    // achieves F1 ~0.95 on CDnet 2014 — best non-DNN BGS algorithm.
    //
    // note: this is a "quality mode" algorithm — not real-time at 1080p on CPU.
    // suitable for offline analysis, downscaled frames, or selected ROIs.
    //
    // ref: St-Charles, Bhatt, Bhatt, "Flexible Background Subtraction With
    // Self-Balanced Local Sensitivity" (CVPR 2014 workshop)
    //
    // simplified from LITIV framework (Apache 2.0), adapted to spclib pattern.
    // this implementation does NOT use parallel partitioning because LBSP
    // requires a 5x5 spatial neighborhood — partitions would need overlap handling.
    class SuBSENSE final
        : public CoreBgs
    {
    public:
        SuBSENSE(SuBSENSEParams _params = SuBSENSEParams(), size_t _num_processes_parallel = 1);
        ~SuBSENSE();

        SuBSENSEParams& get_parameters() override { return m_params; }

        void get_background_image(cv::Mat& _bgImage) override;

    private:
        void initialize(const cv::Mat& _image) override;
        void process(const cv::Mat& _image, cv::Mat& _fgmask, const cv::Mat& _detectMask, int _num_process) override;

        SuBSENSEParams m_params;

        int m_width;
        int m_height;
        int m_channels;
        int m_stride;

        // per-pixel adaptive parameters
        std::vector<float> m_color_thresholds;    // [num_pixels] adaptive color distance threshold
        std::vector<float> m_desc_thresholds;     // [num_pixels] adaptive LBSP hamming distance threshold
        std::vector<float> m_learning_rates;      // [num_pixels] adaptive update probability
        std::vector<float> m_blink_counts;        // [num_pixels] counts rapid fg/bg transitions

        // background model: N samples per pixel
        // for mono: bg_colors[sample * num_pixels + pixel]
        // for color: bg_colors[(sample * num_pixels + pixel) * 3 + c]
        std::vector<uint8_t> m_bg_colors;
        // LBSP descriptors per sample (mono: 1x uint16, color: 3x uint16)
        std::vector<uint16_t> m_bg_descs;

        // previous frame mask for feedback
        std::vector<uint8_t> m_prev_fg_mask;

        // frame counter for feedback delay
        int m_frame_count;

        std::mt19937 m_rng;

        void process_mono(const uint8_t* _input, int _stride, const uint8_t* _detect_mask, uint8_t* _output);
        void process_color(const uint8_t* _input, int _stride, const uint8_t* _detect_mask, uint8_t* _output);

        void update_adaptive_params();
    };
}
