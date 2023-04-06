/****************************************************************************
 *
 *   Copyright (C) 2012, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mathlib.h
 *
 * Target specific math functions and definitions
 *
 * @author Siddharth Bharat Purohit <siddharthbharatpurohit@gmail.com>
 */
#ifndef MATHLIB_H
#define MATHLIB_H
#include "matrix/math.hpp"
#include <float.h>
#include <semaphore.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#ifdef ECL_STANDALONE

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

#ifndef M_PI_2_F
#define M_PI_2_F (M_PI / 2.0f)
#endif

#ifndef M_PI
#define M_PI 3.141592653589793238462643383280
#endif

namespace math
{
template <typename Type>
static constexpr Type min(Type val1, Type val2)
{
	return (val1 < val2) ? val1 : val2;
}

template <typename Type>
static constexpr Type max(Type val1, Type val2)
{
	return (val1 > val2) ? val1 : val2;
}
template<typename _Tp>
constexpr _Tp max(_Tp a, _Tp b, _Tp c)
{
	return max(max(a, b), c);
}

template <typename Type>
static constexpr Type constrain(Type val, Type min, Type max)
{
	return (val < min) ? min : ((val > max) ? max : val);
}

template <typename Type>
static constexpr Type radians(Type degrees)
{
	return (degrees / Type(180)) * Type(M_PI);
}

template <typename Type>
static constexpr Type degrees(Type radians)
{
	return (radians * Type(180)) / Type(M_PI);
}

}  // namespace math

static inline constexpr bool PX4_ISFINITE(float x) { return __builtin_isfinite(x); }
static inline constexpr bool PX4_ISFINITE(double x) { return __builtin_isfinite(x); }

inline bool isFinite(const float &value)
{
	return PX4_ISFINITE(value);
}

inline bool isFinite(const matrix::Vector3f &value)
{
	return PX4_ISFINITE(value(0)) && PX4_ISFINITE(value(1)) && PX4_ISFINITE(value(2));
}
template <typename T>
class AlphaFilter
{
public:
	AlphaFilter() = default;
	explicit AlphaFilter(float alpha) : _alpha(alpha) {}

	~AlphaFilter() = default;

	/**
	 * Set filter parameters for time abstraction
	 *
	 * Both parameters have to be provided in the same units.
	 *
	 * @param sample_interval interval between two samples
	 * @param time_constant filter time constant determining convergence
	 */
	void setParameters(float sample_interval, float time_constant)
	{
		const float denominator = time_constant + sample_interval;

		if (denominator > FLT_EPSILON) {
			setAlpha(sample_interval / denominator);
		}
	}

	bool setCutoffFreq(float sample_freq, float cutoff_freq)
	{
		if ((sample_freq <= 0.f) || (cutoff_freq <= 0.f) || (cutoff_freq >= sample_freq / 2.f)
		    || !isFinite(sample_freq) || !isFinite(cutoff_freq)) {

			// Invalid parameters
			return false;
		}

		setParameters(1.f / sample_freq, 1.f / (2.f * M_PI_F * cutoff_freq));
		_cutoff_freq = cutoff_freq;
		return true;
	}

	/**
	 * Set filter parameter alpha directly without time abstraction
	 *
	 * @param alpha [0,1] filter weight for the previous state. High value - long time constant.
	 */
	void setAlpha(float alpha) { _alpha = alpha; }

	/**
	 * Set filter state to an initial value
	 *
	 * @param sample new initial value
	 */
	void reset(const T &sample) { _filter_state = sample; }

	/**
	 * Add a new raw value to the filter
	 *
	 * @return retrieve the filtered result
	 */
	const T &update(const T &sample)
	{
		_filter_state = updateCalculation(sample);
		return _filter_state;
	}

	const T &getState() const { return _filter_state; }
	float getCutoffFreq() const { return _cutoff_freq; }

protected:
	T updateCalculation(const T &sample) { return (1.f - _alpha) * _filter_state + _alpha * sample; }

	float _cutoff_freq{0.f};
	float _alpha{0.f};
	T _filter_state{};
};


#else

#include <mathlib/mathlib.h>

#endif  // ECL_STANDALONE
#endif  // MATHLIB_H
