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

#include <bgs/CoreParameters.hpp>

#include <algorithm>

namespace spclib::bgs
{
    class SuBSENSEParams
        : public CoreParameters
    {
    public:
        static inline const int32_t DEFAULT_BG_SAMPLES{50};
        static inline const int32_t DEFAULT_REQUIRED_MATCHES{2};
        static inline const float DEFAULT_INITIAL_COLOR_THRESHOLD{30.0f};
        static inline const int32_t DEFAULT_INITIAL_DESC_THRESHOLD{3};
        static inline const float DEFAULT_LEARNING_RATE_LOWER{0.01f};
        static inline const float DEFAULT_LEARNING_RATE_UPPER{0.1f};

        SuBSENSEParams()
            : CoreParameters()
        {
            set_bg_samples(DEFAULT_BG_SAMPLES);
            set_required_matches(DEFAULT_REQUIRED_MATCHES);
            set_initial_color_threshold(DEFAULT_INITIAL_COLOR_THRESHOLD);
            set_initial_desc_threshold(DEFAULT_INITIAL_DESC_THRESHOLD);
            set_learning_rate_lower(DEFAULT_LEARNING_RATE_LOWER);
            set_learning_rate_upper(DEFAULT_LEARNING_RATE_UPPER);
        }

        SuBSENSEParams(const SuBSENSEParams& _p)
            : CoreParameters()
        {
            set_bg_samples(_p.bg_samples);
            set_required_matches(_p.required_matches);
            set_initial_color_threshold(_p.initial_color_threshold);
            set_initial_desc_threshold(_p.initial_desc_threshold);
            set_learning_rate_lower(_p.learning_rate_lower);
            set_learning_rate_upper(_p.learning_rate_upper);
        }

        int32_t get_bg_samples() const { return bg_samples; }
        int32_t get_required_matches() const { return required_matches; }
        float get_initial_color_threshold() const { return initial_color_threshold; }
        int32_t get_initial_desc_threshold() const { return initial_desc_threshold; }
        float get_learning_rate_lower() const { return learning_rate_lower; }
        float get_learning_rate_upper() const { return learning_rate_upper; }

        void set_bg_samples(int32_t _v)
        {
            int32_t clamped = std::clamp(_v, 10, 100);
            if (clamped == bg_samples) return;
            bg_samples = clamped;
            if (m_core_bgs) m_core_bgs->restart();
        }

        void set_required_matches(int32_t _v)
        {
            required_matches = std::clamp(_v, 1, 10);
        }

        void set_initial_color_threshold(float _v)
        {
            initial_color_threshold = std::clamp(_v, 1.0f, 255.0f);
        }

        void set_initial_desc_threshold(int32_t _v)
        {
            initial_desc_threshold = std::clamp(_v, 1, 16);
        }

        void set_learning_rate_lower(float _v)
        {
            learning_rate_lower = std::clamp(_v, 0.001f, 1.0f);
        }

        void set_learning_rate_upper(float _v)
        {
            learning_rate_upper = std::clamp(_v, learning_rate_lower, 1.0f);
        }

        friend class SuBSENSE;

    protected:
        // set_bg_samples() guards on the current value, so the constructors read
        // these before writing them; leaving them indeterminate is UB the
        // optimizer is entitled to fold the whole caller away on.
        int32_t bg_samples{DEFAULT_BG_SAMPLES};
        int32_t required_matches{DEFAULT_REQUIRED_MATCHES};
        float initial_color_threshold{DEFAULT_INITIAL_COLOR_THRESHOLD};
        int32_t initial_desc_threshold{DEFAULT_INITIAL_DESC_THRESHOLD};
        float learning_rate_lower{DEFAULT_LEARNING_RATE_LOWER};
        float learning_rate_upper{DEFAULT_LEARNING_RATE_UPPER};
    };
}
