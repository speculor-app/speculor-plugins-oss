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
 *
 *   The ViBe algorithm is patented in some jurisdictions; see
 *   THIRD_PARTY_NOTICES.md.
 */

#include "Vibe.hpp"

#include <iostream>
#include <execution>

#ifdef _MSC_VER
#define SPC_RESTRICT __restrict
#else
#define SPC_RESTRICT __restrict__
#endif

namespace spclib::bgs
{
    Vibe::Vibe(VibeParams _params, size_t _num_processes_parallel)
        : CoreBgs(_num_processes_parallel), m_params(_params)
    {
    }

    Vibe::~Vibe()
    {
    }

    void Vibe::initialize(const cv::Mat &_init_img)
    {
        auto &init_img = _init_img;
        std::vector<std::unique_ptr<Img>> img_split(m_num_processes_parallel);
        m_orig_img_size = ImgSize::create(init_img);
        Img frameImg(init_img.data, *m_orig_img_size);
        split_img(frameImg, img_split, static_cast<int>(m_num_processes_parallel));

        m_random_generators.resize(m_num_processes_parallel);
        m_bg_img_samples.resize(m_num_processes_parallel);
        if (m_orig_img_size->bytes_per_channel == 1)
        {
            for (size_t i{0}; i < m_num_processes_parallel; ++i)
            {
                initialize<uint8_t>(*img_split[i], m_bg_img_samples[i], m_random_generators[i]);
            }
        }
        else
        {
            for (size_t i{0}; i < m_num_processes_parallel; ++i)
            {
                initialize<uint16_t>(*img_split[i], m_bg_img_samples[i], m_random_generators[i]);
            }
        }
    }

    template <class T>
    void Vibe::initialize(const Img &_init_img, std::vector<std::unique_ptr<Img>> &_bg_img_samples, Pcg32 &_rnd_gen)
    {
        int y_sample, x_sample;
        _bg_img_samples.resize(m_params.bg_samples);
        for (size_t s{0}; s < m_params.bg_samples; ++s)
        {
            _bg_img_samples[s] = Img::create(_init_img.size, false);
            for (int y_orig{0}; y_orig < _init_img.size.height; y_orig++)
            {
                for (int x_orig{0}; x_orig < _init_img.size.width; x_orig++)
                {
                    get_sample_position_7x7_std2(_rnd_gen.fast(), x_sample, y_sample, x_orig, y_orig, _init_img.size);
                    const size_t pixel_pos = (y_orig * _init_img.size.width + x_orig) * _init_img.size.num_channels;
                    const size_t sample_pos = (y_sample * _init_img.size.width + x_sample) * _init_img.size.num_channels;
                    _bg_img_samples[s]->ptr<T>()[pixel_pos] = _init_img.ptr<T>()[sample_pos];
                    if (_init_img.size.num_channels > 1)
                    {
                        _bg_img_samples[s]->ptr<T>()[pixel_pos + 1] = _init_img.ptr<T>()[sample_pos + 1];
                        _bg_img_samples[s]->ptr<T>()[pixel_pos + 2] = _init_img.ptr<T>()[sample_pos + 2];
                    }
                }
            }
        }
    }

    void Vibe::process(const cv::Mat &_image, cv::Mat &_fg_mask, const cv::Mat &_detect_mask, int _num_process)
    {
        // const_casts safe: Img wraps these read-only (Img ctor requires uint8_t*)
        Img img_split(const_cast<uint8_t*>(_image.data), ImgSize(_image));
        Img mask_partial(_fg_mask.data, ImgSize(_fg_mask));
        Img detect_mask_partial(const_cast<uint8_t*>(_detect_mask.data), ImgSize(_detect_mask));
        if (img_split.size.num_channels > 1)
        {
            if (img_split.size.bytes_per_channel == 1)
            {
                apply3<uint8_t>(img_split, mask_partial, detect_mask_partial, _num_process);
            }
            else
            {
                apply3<uint16_t>(img_split, mask_partial, detect_mask_partial, _num_process);
            }
        }
        else
        {
            if (img_split.size.bytes_per_channel == 1)
            {
                apply1<uint8_t>(img_split, mask_partial, detect_mask_partial, _num_process);
            }
            else
            {
                apply1<uint16_t>(img_split, mask_partial, detect_mask_partial, _num_process);
            }
        }
    }

    template <class T>
    void Vibe::apply3(const Img &_image,
                      Img &_fg_mask,
                      const Img &_detect_mask,
                      int _num_process)
    {
        auto &_bg_img = m_bg_img_samples[_num_process];
        auto &_rnd_gen = m_random_generators[_num_process];
        const auto has_detect_mask = !_detect_mask.empty();

        _fg_mask.clear();

        const auto n_color_dist_threshold = sizeof(T) == 1
            ? static_cast<int64_t>(m_params.threshold_color_squared)
            : static_cast<int64_t>(m_params.threshold_color16_squared);
        const size_t n_samples = m_params.bg_samples;
        const uint32_t required = m_params.required_bg_samples;
        const uint32_t and_lr = m_params.and_learning_rate;
        const uint32_t and_bg = m_params.and_bg_samples;

        // stack-allocate background sample pointers (avoid per-frame heap allocation)
        static constexpr size_t MAX_BG_SAMPLES = 64;
        T *bg_ptrs[MAX_BG_SAMPLES];
        for (size_t s = 0; s < n_samples; ++s)
            bg_ptrs[s] = _bg_img[s]->ptr<T>();

        const T * SPC_RESTRICT img_base = _image.ptr<T>();
        uint8_t * SPC_RESTRICT fg_ptr = _fg_mask.data;

        size_t pix_offset{0}, color_pix_offset{0};
        for (int y{0}; y < _image.size.height; ++y)
        {
            for (int x{0}; x < _image.size.width; ++x, ++pix_offset, color_pix_offset += _image.size.num_channels)
            {
                if (has_detect_mask && (_detect_mask.data[pix_offset] == 0))
                {
                    continue;
                }

                size_t n_good_samples_count{0},
                    n_sample_idx{0};

                const T *const pix_data{&img_base[color_pix_offset]};

                while (n_sample_idx < n_samples)
                {
                    const T *const bg{&bg_ptrs[n_sample_idx][color_pix_offset]};
                    if (l2_dist3_squared(pix_data, bg) < n_color_dist_threshold)
                    {
                        ++n_good_samples_count;
                        if (n_good_samples_count >= required)
                        {
                            break;
                        }
                    }
                    ++n_sample_idx;
                }
                if (n_good_samples_count < required) [[unlikely]]
                {
                    fg_ptr[pix_offset] = UCHAR_MAX;
                }
                else
                {
                    // Draws 1, 3 and 4 are consumed only by branches taken once
                    // per `learning_rate` pixels, so fetching all five up front
                    // discarded most of them -- five table loads on every
                    // background pixel, which is the bulk of them. peek() the
                    // ones a branch actually takes and skip() the whole block:
                    // the stream advances exactly as five fast() calls would, so
                    // the model evolves identically.
                    const uint32_t r0 = _rnd_gen.peek(0);
                    const uint32_t r2 = _rnd_gen.peek(2);

                    if ((r0 & and_lr) == 0)
                    {
                        T *const bg_img_pix_data{&bg_ptrs[_rnd_gen.peek(1) & and_bg][color_pix_offset]};
                        bg_img_pix_data[0] = pix_data[0];
                        bg_img_pix_data[1] = pix_data[1];
                        bg_img_pix_data[2] = pix_data[2];
                    }
                    if ((r2 & and_lr) == 0)
                    {
                        const int neigh_data{get_neighbor_position_3x3(x, y, _image.size, _rnd_gen.peek(3)) * 3};
                        T *const xy_rand_data{&bg_ptrs[_rnd_gen.peek(4) & and_bg][neigh_data]};
                        xy_rand_data[0] = pix_data[0];
                        xy_rand_data[1] = pix_data[1];
                        xy_rand_data[2] = pix_data[2];
                    }
                    _rnd_gen.skip(5);
                }
            }
        }
    }

    template <class T>
    void Vibe::apply1(const Img &_image,
                      Img &_fg_mask,
                      const Img &_detect_mask,
                      int _num_process)
    {
        auto &_bg_img = m_bg_img_samples[_num_process];
        auto &_rnd_gen = m_random_generators[_num_process];
        const auto has_detect_mask = !_detect_mask.empty();

        const T * SPC_RESTRICT img_ptr = _image.ptr<T>();
        const uint8_t *mask_ptr = has_detect_mask ? _detect_mask.data : nullptr;
        uint8_t * SPC_RESTRICT fg_ptr = _fg_mask.data;
        const size_t n_samples = m_params.bg_samples;
        const size_t num_pixels = _image.size.num_pixels;
        const int width = _image.size.width;
        const int height = _image.size.height;

        // stack-allocate background sample pointers (avoid per-frame heap allocation)
        static constexpr size_t MAX_BG_SAMPLES = 64;
        T *bg_ptrs[MAX_BG_SAMPLES];
        for (size_t s = 0; s < n_samples; ++s)
            bg_ptrs[s] = _bg_img[s]->ptr<T>();

        // saturate threshold to T's range for type-native SIMD-friendly comparison
        // (values exceeding type range mean "everything matches" — semantically correct)
        const T threshold_native = sizeof(T) == 1
            ? static_cast<T>(m_params.threshold_mono > 255u ? 255u : m_params.threshold_mono)
            : static_cast<T>(m_params.threshold_mono16 > 65535u ? 65535u : m_params.threshold_mono16);

        // Pass 1: Count matching samples per pixel (sample-major for contiguous access).
        // Uses type-native arithmetic so the compiler can vectorize at full register width
        // (32 uint8 per AVX2 vs 8 with int32 widening — 4x throughput).
        _fg_mask.clear();
        for (size_t s = 0; s < n_samples; ++s)
        {
            const T * SPC_RESTRICT bg = bg_ptrs[s];
            for (size_t i = 0; i < num_pixels; ++i)
            {
                const T a = img_ptr[i], b = bg[i];
                const T diff = (a > b) ? static_cast<T>(a - b) : static_cast<T>(b - a);
                fg_ptr[i] += (diff < threshold_native);
            }
        }

        // Pass 2: Classify pixels and update background model (sequential due to RNG)
        const uint32_t required = m_params.required_bg_samples;
        const uint32_t and_lr = m_params.and_learning_rate;
        const uint32_t and_bg = m_params.and_bg_samples;
        size_t pix_offset = 0;
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x, ++pix_offset)
            {
                if (has_detect_mask && (mask_ptr[pix_offset] == 0))
                {
                    fg_ptr[pix_offset] = 0;
                    continue;
                }
                if (fg_ptr[pix_offset] < required) [[unlikely]]
                {
                    fg_ptr[pix_offset] = UCHAR_MAX;
                }
                else
                {
                    fg_ptr[pix_offset] = 0;
                    const T pix_data = img_ptr[pix_offset];

                    // Draws 1, 3 and 4 are consumed only by branches taken once
                    // per `learning_rate` pixels; peek() those and skip() the
                    // block so the stream still advances by five, leaving the
                    // model evolution identical without the discarded loads.
                    const uint32_t r0 = _rnd_gen.peek(0);
                    const uint32_t r2 = _rnd_gen.peek(2);

                    if ((r0 & and_lr) == 0)
                    {
                        bg_ptrs[_rnd_gen.peek(1) & and_bg][pix_offset] = pix_data;
                    }
                    if ((r2 & and_lr) == 0)
                    {
                        const int neigh_data{get_neighbor_position_3x3(x, y, _image.size, _rnd_gen.peek(3))};
                        bg_ptrs[_rnd_gen.peek(4) & and_bg][neigh_data] = pix_data;
                    }
                    _rnd_gen.skip(5);
                }
            }
        }
    }

    void Vibe::get_background_image(cv::Mat &_bg_image)
    {
        cv::Mat avg_bg_img = cv::Mat::zeros(m_orig_img_size->height, m_orig_img_size->width, CV_32FC(m_orig_img_size->num_channels));

        const float inv_bg_samples = 1.0f / static_cast<float>(m_params.bg_samples);

        for (size_t t{0}; t < m_num_processes_parallel; ++t)
        {
            const std::vector<std::unique_ptr<Img>> &bg_samples = m_bg_img_samples[t];
            for (size_t n{0}; n < m_params.bg_samples; ++n)
            {
                size_t in_pix_offset{0};
                size_t out_pix_offset{bg_samples[0]->size.original_pixel_pos * sizeof(float) * bg_samples[0]->size.num_channels};
                // step by bytes-per-pixel (num_channels * bytes_per_channel),
                // not element count: data is a raw byte buffer sized in bytes,
                // so for 16-bit input the old element-count stride over-iterated
                // ~2x and wrote past avg_bg_img (heap OOB).
                const size_t bpc = static_cast<size_t>(m_orig_img_size->bytes_per_channel);
                for (; in_pix_offset < bg_samples[n]->size.size_in_bytes;
                     in_pix_offset += m_orig_img_size->num_channels * bpc,
                     out_pix_offset += sizeof(float) * bg_samples[0]->size.num_channels)
                {
                    const uint8_t *const pix_data{&bg_samples[n]->data[in_pix_offset]};
                    float *const out_data{(float *)(avg_bg_img.data + out_pix_offset)};
                    for (int c{0}; c < m_orig_img_size->num_channels; ++c)
                    {
                        const float v{(bpc == 2)
                            ? static_cast<float>(reinterpret_cast<const uint16_t *>(pix_data)[c])
                            : static_cast<float>(pix_data[c])};
                        out_data[c] += v * inv_bg_samples;
                    }
                }
            }
        }

        avg_bg_img.convertTo(_bg_image, CV_8U);
    }
}