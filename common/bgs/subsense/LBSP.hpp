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

#include <cstdint>
#include <cstddef>

namespace spclib::bgs
{
    // Local Binary Similarity Pattern (LBSP) computation
    // LBSP compares the center pixel against its neighbors using the center
    // pixel value as the reference, producing a 16-bit descriptor.
    // Unlike LBP which uses neighbor-to-neighbor comparisons, LBSP uses
    // absolute differences against a threshold relative to the center value.
    //
    // the 16-bit pattern samples a 5x5 neighborhood (16 of 24 neighbors):
    //     . x . x .
    //     x x x x x
    //     . x C x .
    //     x x x x x
    //     . x . x .
    //
    // ref: St-Charles & Bhatt, "Flexible Background Subtraction With
    // Self-Balanced Local Sensitivity" (CVPR 2014 workshop)
    namespace lbsp
    {
        // 16 neighbor offsets (row, col) relative to center in a 5x5 pattern
        static constexpr int OFFSETS[16][2] = {
            {-2, -1}, {-2,  1},                         // row -2
            {-1, -2}, {-1, -1}, {-1, 0}, {-1, 1}, {-1, 2}, // row -1
            { 0, -2}, { 0, -1},           { 0, 1}, { 0, 2}, // row 0 (skip center)
            { 1, -2}, { 1, -1}, { 1, 0}, { 1, 1}, { 1, 2}, // row +1
        };
        // note: only 16 positions used to produce a 16-bit descriptor

        // compute LBSP descriptor for a single grayscale pixel
        // _ref is the reference intensity (often the bg model mean, not current pixel)
        // _threshold is the absolute difference threshold
        inline uint16_t compute_mono(const uint8_t* _img, int _stride, int _x, int _y,
                                     int _width, int _height, uint8_t _ref, int _threshold)
        {
            uint16_t desc = 0;
            for (int b = 0; b < 16; ++b)
            {
                int ny = _y + OFFSETS[b][0];
                int nx = _x + OFFSETS[b][1];
                // clamp to image bounds
                if (ny < 0) ny = 0;
                else if (ny >= _height) ny = _height - 1;
                if (nx < 0) nx = 0;
                else if (nx >= _width) nx = _width - 1;

                uint8_t neighbor = _img[ny * _stride + nx];
                int diff = static_cast<int>(neighbor) - static_cast<int>(_ref);
                if (diff < 0) diff = -diff;
                if (diff <= _threshold)
                    desc |= (1u << b);
            }
            return desc;
        }

        // compute LBSP descriptor for a 3-channel color pixel
        // returns 3 x 16-bit descriptors (one per channel), packed sequentially
        inline void compute_color(const uint8_t* _img, int _stride, int _x, int _y,
                                  int _width, int _height, const uint8_t _ref[3],
                                  int _threshold, uint16_t _desc[3])
        {
            _desc[0] = 0;
            _desc[1] = 0;
            _desc[2] = 0;

            for (int b = 0; b < 16; ++b)
            {
                int ny = _y + OFFSETS[b][0];
                int nx = _x + OFFSETS[b][1];
                if (ny < 0) ny = 0;
                else if (ny >= _height) ny = _height - 1;
                if (nx < 0) nx = 0;
                else if (nx >= _width) nx = _width - 1;

                const uint8_t* neighbor = _img + ny * _stride + nx * 3;
                for (int c = 0; c < 3; ++c)
                {
                    int diff = static_cast<int>(neighbor[c]) - static_cast<int>(_ref[c]);
                    if (diff < 0) diff = -diff;
                    if (diff <= _threshold)
                        _desc[c] |= (1u << b);
                }
            }
        }

        // hamming distance between two 16-bit descriptors
        inline int hamming(uint16_t _a, uint16_t _b)
        {
            uint16_t x = _a ^ _b;
            // popcount
            int count = 0;
            while (x)
            {
                count += (x & 1);
                x >>= 1;
            }
            return count;
        }

        // combined color+LBSP distance
        // returns the number of channels where both color AND texture differ
        inline int color_lbsp_distance_mono(uint8_t _pixel, uint8_t _bg,
                                            uint16_t _desc, uint16_t _bg_desc,
                                            int _color_threshold, int _desc_threshold)
        {
            int color_diff = static_cast<int>(_pixel) - static_cast<int>(_bg);
            if (color_diff < 0) color_diff = -color_diff;
            int desc_dist = hamming(_desc, _bg_desc);

            return (color_diff > _color_threshold || desc_dist > _desc_threshold) ? 1 : 0;
        }

        inline int color_lbsp_distance_color(const uint8_t _pixel[3], const uint8_t _bg[3],
                                             const uint16_t _desc[3], const uint16_t _bg_desc[3],
                                             int _color_threshold, int _desc_threshold)
        {
            int mismatches = 0;
            for (int c = 0; c < 3; ++c)
            {
                int color_diff = static_cast<int>(_pixel[c]) - static_cast<int>(_bg[c]);
                if (color_diff < 0) color_diff = -color_diff;
                int desc_dist = hamming(_desc[c], _bg_desc[c]);
                if (color_diff > _color_threshold || desc_dist > _desc_threshold)
                    ++mismatches;
            }
            return mismatches;
        }
    }
}
