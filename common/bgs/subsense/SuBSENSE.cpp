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

#include "SuBSENSE.hpp"

#include <cmath>
#include <algorithm>
#include <cstring>

static const uint8_t ZERO_UC{0};

namespace spclib::bgs
{
    SuBSENSE::SuBSENSE(SuBSENSEParams _params, size_t /*_num_processes_parallel*/)
        : CoreBgs(1) // force single-threaded: LBSP needs spatial neighborhood
        , m_params{_params}
        , m_width{0}
        , m_height{0}
        , m_channels{0}
        , m_stride{0}
        , m_frame_count{0}
        , m_rng{42}
    {
        m_params.set_bgs(this);
    }

    SuBSENSE::~SuBSENSE()
    {
    }

    void SuBSENSE::get_background_image(cv::Mat& _bgImage)
    {
        if (m_bg_colors.empty() || !m_initialized) return;

        size_t np = static_cast<size_t>(m_width) * m_height;

        if (m_channels == 1)
        {
            _bgImage.create(m_height, m_width, CV_8UC1);
            // use first sample as background estimate
            std::memcpy(_bgImage.data, m_bg_colors.data(), np);
        }
        else
        {
            _bgImage.create(m_height, m_width, CV_8UC3);
            std::memcpy(_bgImage.data, m_bg_colors.data(), np * 3);
        }
    }

    void SuBSENSE::initialize(const cv::Mat& _image)
    {
        auto& img = _image;
        m_width = img.cols;
        m_height = img.rows;
        m_channels = img.channels();
        m_stride = static_cast<int>(img.step[0]);

        size_t np = static_cast<size_t>(m_width) * m_height;
        int N = m_params.bg_samples;

        m_color_thresholds.assign(np, m_params.initial_color_threshold);
        m_desc_thresholds.assign(np, static_cast<float>(m_params.initial_desc_threshold));
        m_learning_rates.assign(np, (m_params.learning_rate_lower + m_params.learning_rate_upper) * 0.5f);
        m_blink_counts.assign(np, 0.0f);
        m_prev_fg_mask.assign(np, 0);

        m_bg_colors.resize(N * np * m_channels);
        m_bg_descs.resize(N * np * m_channels); // for color: 3 descriptors per pixel per sample

        // An LBSP descriptor is a function of the position and the frame alone,
        // so it is the same whichever sample lands on that position. Computing
        // it once per pixel instead of once per (sample, pixel) does the 16
        // neighbour comparisons N times fewer -- at 4K with the default 50
        // samples that is the difference between ~45 s and under a second, and
        // 45 s is long enough for the node to outlive a pipeline stop and read
        // its input frame after the engine has freed it.
        const int desc_threshold = static_cast<int>(m_params.initial_color_threshold * 0.3f);
        std::vector<uint16_t> desc_map(np * m_channels);
        for (int y = 0; y < m_height; ++y)
        {
            for (int x = 0; x < m_width; ++x)
            {
                const size_t idx = static_cast<size_t>(y) * m_width + x;
                if (m_channels == 1)
                {
                    desc_map[idx] = lbsp::compute_mono(img.data, m_stride, x, y,
                                                       m_width, m_height,
                                                       img.data[y * m_stride + x],
                                                       desc_threshold);
                }
                else
                {
                    lbsp::compute_color(img.data, m_stride, x, y, m_width, m_height,
                                        img.data + y * m_stride + x * 3,
                                        desc_threshold, &desc_map[idx * 3]);
                }
            }
        }

        // initialize all N samples from the first frame with spatial jitter
        std::uniform_int_distribution<int> jitter_dist(-2, 2);

        for (int s = 0; s < N; ++s)
        {
            for (int y = 0; y < m_height; ++y)
            {
                for (int x = 0; x < m_width; ++x)
                {
                    size_t pixel_idx = static_cast<size_t>(y) * m_width + x;

                    // jittered sample position for diversity
                    int jy = std::clamp(y + jitter_dist(m_rng), 0, m_height - 1);
                    int jx = std::clamp(x + jitter_dist(m_rng), 0, m_width - 1);

                    const size_t j_idx = static_cast<size_t>(jy) * m_width + jx;

                    if (m_channels == 1)
                    {
                        m_bg_colors[s * np + pixel_idx] = img.data[jy * m_stride + jx];
                        m_bg_descs[s * np + pixel_idx] = desc_map[j_idx];
                    }
                    else
                    {
                        const uint8_t* src = img.data + jy * m_stride + jx * 3;
                        size_t color_base = (s * np + pixel_idx) * 3;
                        m_bg_colors[color_base + 0] = src[0];
                        m_bg_colors[color_base + 1] = src[1];
                        m_bg_colors[color_base + 2] = src[2];

                        size_t desc_base = (s * np + pixel_idx) * 3;
                        m_bg_descs[desc_base + 0] = desc_map[j_idx * 3 + 0];
                        m_bg_descs[desc_base + 1] = desc_map[j_idx * 3 + 1];
                        m_bg_descs[desc_base + 2] = desc_map[j_idx * 3 + 2];
                    }
                }
            }
        }

        m_frame_count = 0;
    }

    void SuBSENSE::process(const cv::Mat& _img_input, cv::Mat& _img_output,
                           const cv::Mat& _detectMask, int)
    {
        auto& img_input = _img_input;
        if (_img_output.empty())
        {
            _img_output.create(img_input.size(), CV_8UC1);
        }
        auto& img_output = _img_output;

        const uint8_t* detect_mask = _detectMask.empty() ? nullptr : _detectMask.data;
        int stride = static_cast<int>(img_input.step[0]);

        if (m_channels == 1)
            process_mono(img_input.data, stride, detect_mask, img_output.data);
        else
            process_color(img_input.data, stride, detect_mask, img_output.data);

        // run feedback loop to adapt per-pixel parameters
        ++m_frame_count;
        if (m_frame_count > 5)
        {
            update_adaptive_params();
        }

        // save current mask for next frame's feedback
        size_t np = static_cast<size_t>(m_width) * m_height;
        std::memcpy(m_prev_fg_mask.data(), img_output.data, np);
    }

    void SuBSENSE::process_mono(const uint8_t* _input, int _stride,
                                const uint8_t* _detect_mask, uint8_t* _output)
    {
        size_t np = static_cast<size_t>(m_width) * m_height;
        int N = m_params.bg_samples;
        int req_matches = m_params.required_matches;

        std::uniform_int_distribution<int> sample_dist(0, N - 1);
        std::uniform_int_distribution<int> neighbor_dist(-1, 1);
        std::uniform_real_distribution<float> prob_dist(0.0f, 1.0f);

        for (int y = 0; y < m_height; ++y)
        {
            for (int x = 0; x < m_width; ++x)
            {
                size_t idx = static_cast<size_t>(y) * m_width + x;

                if (_detect_mask && (_detect_mask[idx] == 0))
                {
                    _output[idx] = ZERO_UC;
                    continue;
                }

                uint8_t pixel = _input[y * _stride + x];

                // compute LBSP for current pixel using pixel value as reference
                int lbsp_thresh = static_cast<int>(m_color_thresholds[idx] * 0.3f);
                uint16_t curr_desc = lbsp::compute_mono(_input, _stride, x, y,
                                                        m_width, m_height, pixel, lbsp_thresh);

                float color_thresh = m_color_thresholds[idx];
                float desc_thresh = m_desc_thresholds[idx];

                int matches = 0;
                for (int s = 0; s < N && matches < req_matches; ++s)
                {
                    uint8_t bg_val = m_bg_colors[s * np + idx];
                    uint16_t bg_desc = m_bg_descs[s * np + idx];

                    int dist = lbsp::color_lbsp_distance_mono(
                        pixel, bg_val, curr_desc, bg_desc,
                        static_cast<int>(color_thresh),
                        static_cast<int>(desc_thresh));
                    if (dist == 0)
                        ++matches;
                }

                bool is_bg = (matches >= req_matches);
                _output[idx] = is_bg ? ZERO_UC : UCHAR_MAX;

                // update background model if classified as background
                if (is_bg)
                {
                    float lr = m_learning_rates[idx];
                    if (prob_dist(m_rng) < lr)
                    {
                        int replace_idx = sample_dist(m_rng);
                        m_bg_colors[replace_idx * np + idx] = pixel;
                        m_bg_descs[replace_idx * np + idx] = curr_desc;
                    }

                    // propagate to random neighbor (spatial consistency)
                    if (prob_dist(m_rng) < lr)
                    {
                        int ny = std::clamp(y + neighbor_dist(m_rng), 0, m_height - 1);
                        int nx = std::clamp(x + neighbor_dist(m_rng), 0, m_width - 1);
                        size_t n_idx = static_cast<size_t>(ny) * m_width + nx;
                        int replace_idx = sample_dist(m_rng);
                        m_bg_colors[replace_idx * np + n_idx] = pixel;
                        m_bg_descs[replace_idx * np + n_idx] = curr_desc;
                    }
                }
            }
        }
    }

    void SuBSENSE::process_color(const uint8_t* _input, int _stride,
                                 const uint8_t* _detect_mask, uint8_t* _output)
    {
        size_t np = static_cast<size_t>(m_width) * m_height;
        int N = m_params.bg_samples;
        int req_matches = m_params.required_matches;

        std::uniform_int_distribution<int> sample_dist(0, N - 1);
        std::uniform_int_distribution<int> neighbor_dist(-1, 1);
        std::uniform_real_distribution<float> prob_dist(0.0f, 1.0f);

        for (int y = 0; y < m_height; ++y)
        {
            for (int x = 0; x < m_width; ++x)
            {
                size_t idx = static_cast<size_t>(y) * m_width + x;

                if (_detect_mask && (_detect_mask[idx] == 0))
                {
                    _output[idx] = ZERO_UC;
                    continue;
                }

                const uint8_t* pixel = _input + y * _stride + x * 3;

                int lbsp_thresh = static_cast<int>(m_color_thresholds[idx] * 0.3f);
                uint16_t curr_desc[3];
                lbsp::compute_color(_input, _stride, x, y,
                                   m_width, m_height, pixel, lbsp_thresh, curr_desc);

                float color_thresh = m_color_thresholds[idx];
                float desc_thresh = m_desc_thresholds[idx];

                int matches = 0;
                for (int s = 0; s < N && matches < req_matches; ++s)
                {
                    size_t color_base = (s * np + idx) * 3;
                    size_t desc_base = (s * np + idx) * 3;
                    const uint8_t bg_val[3] = {
                        m_bg_colors[color_base + 0],
                        m_bg_colors[color_base + 1],
                        m_bg_colors[color_base + 2]
                    };
                    const uint16_t bg_desc[3] = {
                        m_bg_descs[desc_base + 0],
                        m_bg_descs[desc_base + 1],
                        m_bg_descs[desc_base + 2]
                    };

                    // for color: need at least 2 out of 3 channels to match
                    int mismatches = lbsp::color_lbsp_distance_color(
                        pixel, bg_val, curr_desc, bg_desc,
                        static_cast<int>(color_thresh),
                        static_cast<int>(desc_thresh));
                    if (mismatches <= 1)
                        ++matches;
                }

                bool is_bg = (matches >= req_matches);
                _output[idx] = is_bg ? ZERO_UC : UCHAR_MAX;

                if (is_bg)
                {
                    float lr = m_learning_rates[idx];
                    if (prob_dist(m_rng) < lr)
                    {
                        int replace_s = sample_dist(m_rng);
                        size_t color_base = (replace_s * np + idx) * 3;
                        size_t desc_base = (replace_s * np + idx) * 3;
                        m_bg_colors[color_base + 0] = pixel[0];
                        m_bg_colors[color_base + 1] = pixel[1];
                        m_bg_colors[color_base + 2] = pixel[2];
                        m_bg_descs[desc_base + 0] = curr_desc[0];
                        m_bg_descs[desc_base + 1] = curr_desc[1];
                        m_bg_descs[desc_base + 2] = curr_desc[2];
                    }

                    // spatial propagation
                    if (prob_dist(m_rng) < lr)
                    {
                        int ny = std::clamp(y + neighbor_dist(m_rng), 0, m_height - 1);
                        int nx = std::clamp(x + neighbor_dist(m_rng), 0, m_width - 1);
                        size_t n_idx = static_cast<size_t>(ny) * m_width + nx;
                        int replace_s = sample_dist(m_rng);
                        size_t n_color_base = (replace_s * np + n_idx) * 3;
                        size_t n_desc_base = (replace_s * np + n_idx) * 3;
                        m_bg_colors[n_color_base + 0] = pixel[0];
                        m_bg_colors[n_color_base + 1] = pixel[1];
                        m_bg_colors[n_color_base + 2] = pixel[2];
                        m_bg_descs[n_desc_base + 0] = curr_desc[0];
                        m_bg_descs[n_desc_base + 1] = curr_desc[1];
                        m_bg_descs[n_desc_base + 2] = curr_desc[2];
                    }
                }
            }
        }
    }

    // feedback loop: adapt per-pixel thresholds and learning rates
    // based on segmentation stability analysis
    void SuBSENSE::update_adaptive_params()
    {
        size_t np = static_cast<size_t>(m_width) * m_height;
        float lr_lower = m_params.learning_rate_lower;
        float lr_upper = m_params.learning_rate_upper;

        for (size_t i = 0; i < np; ++i)
        {
            bool curr_fg = (m_prev_fg_mask[i] != 0);
            // detect blinking: rapid changes between fg/bg indicate instability
            // m_blink_counts decays, increments on transitions
            float& blink = m_blink_counts[i];
            float& color_t = m_color_thresholds[i];
            float& desc_t = m_desc_thresholds[i];
            float& lr = m_learning_rates[i];

            // if pixel is unstable (blinking), increase thresholds and learning rate
            if (curr_fg)
            {
                blink = blink * 0.95f + 0.05f;
            }
            else
            {
                blink *= 0.9f;
            }

            if (blink > 0.5f)
            {
                // unstable pixel: increase sensitivity thresholds to reduce noise
                color_t = std::min(color_t * 1.02f, 255.0f);
                desc_t = std::min(desc_t * 1.02f, 16.0f);
                lr = std::min(lr * 1.05f, lr_upper);
            }
            else if (blink < 0.1f)
            {
                // stable pixel: tighten thresholds for better sensitivity
                color_t = std::max(color_t * 0.99f, m_params.initial_color_threshold * 0.5f);
                desc_t = std::max(desc_t * 0.99f, 1.0f);
                lr = std::max(lr * 0.98f, lr_lower);
            }
        }
    }
}
