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

#pragma once

#include <bgs/CoreBgs.hpp>
#include <bgs/CoreParameters.hpp>
#include "VibeUtils.hpp"
#include <include/pcg32.hpp>

namespace spclib::bgs
{
    class Vibe
        : public CoreBgs
    {
    public:
        Vibe(VibeParams _params = VibeParams(),
             size_t _numProcessesParallel = DETECT_NUMBER_OF_THREADS);

        ~Vibe();

        VibeParams &get_parameters() override { return m_params; }

        void get_background_image(cv::Mat &_bgImage) override;

    private:
        void initialize(const cv::Mat &_init_img) override;
        void process(const cv::Mat &_image, cv::Mat &_fgmask, const cv::Mat &_detectMask, int _numProcess) override;

        VibeParams m_params;

        std::unique_ptr<ImgSize> m_orig_img_size;
        std::vector<std::vector<std::unique_ptr<Img>>> m_bg_img_samples;
        std::vector<Pcg32> m_random_generators;

        template<class T>
        void initialize(const Img &_initImg, std::vector<std::unique_ptr<Img>> &_bg_img_samples, Pcg32 &_rnd_gen);
        
        template<class T>
        void apply1(const Img &_image, Img &_fg_mask, const Img & _detect_mask, int _num_process);
        template<class T>
        void apply3(const Img &_image, Img &_fg_mask, const Img & _detect_mask, int _num_process);
    };
}