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

#ifdef ECL_STANDALONE
#include "matrix/math.hpp"
#include <float.h>
#include <semaphore.h>
#include <time.h>
#include <string.h>
#include <errno.h>
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
namespace Utilities
{

// return the square of two floating point numbers - used in auto coded sections
static constexpr float sq(float var) { return var * var; }

// converts Tait-Bryan 312 sequence of rotations from frame 1 to frame 2
// to the corresponding rotation matrix that rotates from frame 2 to frame 1
// rot312(0) - First rotation is a RH rotation about the Z axis (rad)
// rot312(1) - Second rotation is a RH rotation about the X axis (rad)
// rot312(2) - Third rotation is a RH rotation about the Y axis (rad)
// See http://www.atacolorado.com/eulersequences.doc
inline matrix::Dcmf taitBryan312ToRotMat(const matrix::Vector3f &rot312)
{
	// Calculate the frame2 to frame 1 rotation matrix from a 312 Tait-Bryan rotation sequence
	const float c2 = cosf(rot312(2)); // third rotation is pitch
	const float s2 = sinf(rot312(2));
	const float s1 = sinf(rot312(1)); // second rotation is roll
	const float c1 = cosf(rot312(1));
	const float s0 = sinf(rot312(0)); // first rotation is yaw
	const float c0 = cosf(rot312(0));

	matrix::Dcmf R;
	R(0, 0) = c0 * c2 - s0 * s1 * s2;
	R(1, 1) = c0 * c1;
	R(2, 2) = c2 * c1;
	R(0, 1) = -c1 * s0;
	R(0, 2) = s2 * c0 + c2 * s1 * s0;
	R(1, 0) = c2 * s0 + s2 * s1 * c0;
	R(1, 2) = s0 * s2 - s1 * c0 * c2;
	R(2, 0) = -s2 * c1;
	R(2, 1) = s1;

	return R;
}

inline matrix::Dcmf quatToInverseRotMat(const matrix::Quatf &quat)
{
	const float q00 = quat(0) * quat(0);
	const float q11 = quat(1) * quat(1);
	const float q22 = quat(2) * quat(2);
	const float q33 = quat(3) * quat(3);
	const float q01 = quat(0) * quat(1);
	const float q02 = quat(0) * quat(2);
	const float q03 = quat(0) * quat(3);
	const float q12 = quat(1) * quat(2);
	const float q13 = quat(1) * quat(3);
	const float q23 = quat(2) * quat(3);

	matrix::Dcmf dcm;
	dcm(0, 0) = q00 + q11 - q22 - q33;
	dcm(1, 1) = q00 - q11 + q22 - q33;
	dcm(2, 2) = q00 - q11 - q22 + q33;
	dcm(1, 0) = 2.0f * (q12 - q03);
	dcm(2, 0) = 2.0f * (q13 + q02);
	dcm(0, 1) = 2.0f * (q12 + q03);
	dcm(2, 1) = 2.0f * (q23 - q01);
	dcm(0, 2) = 2.0f * (q13 - q02);
	dcm(1, 2) = 2.0f * (q23 + q01);

	return dcm;
}

// We should use a 3-2-1 Tait-Bryan (yaw-pitch-roll) rotation sequence
// when there is more roll than pitch tilt and a 3-1-2 rotation sequence
// when there is more pitch than roll tilt to avoid gimbal lock.
inline bool shouldUse321RotationSequence(const matrix::Dcmf &R)
{
	return fabsf(R(2, 0)) < fabsf(R(2, 1));
}

inline float getEuler321Yaw(const matrix::Dcmf &R)
{
	return atan2f(R(1, 0), R(0, 0));
}

inline float getEuler312Yaw(const matrix::Dcmf &R)
{
	return atan2f(-R(0, 1), R(1, 1));
}

inline float getEuler321Yaw(const matrix::Quatf &q)
{
	// Values from yaw_input_321.c file produced by
	// https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/quat2yaw321.m
	const float a = 2.f * (q(0) * q(3) + q(1) * q(2));
	const float b = sq(q(0)) + sq(q(1)) - sq(q(2)) - sq(q(3));
	return atan2f(a, b);
}

inline float getEuler312Yaw(const matrix::Quatf &q)
{
	// Values from yaw_input_312.c file produced by
	// https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/quat2yaw312.m
	const float a = 2.f * (q(0) * q(3) - q(1) * q(2));
	const float b = sq(q(0)) - sq(q(1)) + sq(q(2)) - sq(q(3));
	return atan2f(a, b);
}

inline float getEulerYaw(const matrix::Dcmf &R)
{
	if (shouldUse321RotationSequence(R)) {
		return getEuler321Yaw(R);

	} else {
		return getEuler312Yaw(R);
	}
}

inline matrix::Dcmf updateEuler321YawInRotMat(float yaw, const matrix::Dcmf &rot_in)
{
	matrix::Eulerf euler321(rot_in);
	euler321(2) = yaw;
	return matrix::Dcmf(euler321);
}

inline matrix::Dcmf updateEuler312YawInRotMat(float yaw, const matrix::Dcmf &rot_in)
{
	const matrix::Vector3f rotVec312(yaw,  // yaw
					 asinf(rot_in(2, 1)),  // roll
					 atan2f(-rot_in(2, 0), rot_in(2, 2)));  // pitch
	return taitBryan312ToRotMat(rotVec312);
}

// Checks which euler rotation sequence to use and update yaw in rotation matrix
inline matrix::Dcmf updateYawInRotMat(float yaw, const matrix::Dcmf &rot_in)
{
	if (shouldUse321RotationSequence(rot_in)) {
		return updateEuler321YawInRotMat(yaw, rot_in);

	} else {
		return updateEuler312YawInRotMat(yaw, rot_in);
	}
}

} // namespace Utilities

}  // namespace math

constexpr bool PX4_ISFINITE(float x) { return __builtin_isfinite(x); }
constexpr bool PX4_ISFINITE(double x) { return __builtin_isfinite(x); }


inline bool isFinite(const float &value)
{
	return PX4_ISFINITE(value);
}

inline bool isFinite(const matrix::Vector3f &value)
{
	return value.isAllFinite();
}

using namespace math;

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

#ifndef _STRUCT_TIMESPEC
#define _STRUCT_TIMESPEC        struct timespec

#include <machine/types.h> /* __darwin_time_t */

_STRUCT_TIMESPEC
{
	__darwin_time_t tv_sec;
	long            tv_nsec;
};
#endif /* _STRUCT_TIMESPEC */
/**
 * Absolute time, in microsecond units.
 *
 * Absolute time is measured from some arbitrary epoch shortly after
 * system startup.  It should never wrap or go backwards.
 */
typedef uint64_t	hrt_abstime;
//can't be use to control, just for data flow logi.
#define    NAN          __builtin_nanf("0x7fc00000")

#define PX4_ISFINITE(x) std::isfinite(x) //ISFINITE判断

// User-defined integer literals for different time units.
// The base unit is hrt_abstime in microseconds

constexpr hrt_abstime operator "" _s(unsigned long long seconds)
{
	return hrt_abstime(seconds * 1000000ULL);
}
struct estimator_aid_source1d_s {
	uint64_t timestamp;
	uint64_t timestamp_sample;
	uint64_t time_last_fuse;
	uint32_t device_id;
	float observation;
	float observation_variance;
	float innovation;
	float innovation_variance;
	float test_ratio;
	uint8_t estimator_instance;
	bool fusion_enabled;
	bool innovation_rejected;
	bool fused;
	uint8_t _padding0[4]; // required for logger

};
struct estimator_aid_source2d_s {
	uint64_t timestamp;
	uint64_t timestamp_sample;
	uint64_t time_last_fuse;
	uint32_t device_id;
	float observation[2];
	float observation_variance[2];
	float innovation[2];
	float innovation_variance[2];
	float test_ratio[2];
	uint8_t estimator_instance;
	bool fusion_enabled;
	bool innovation_rejected;
	bool fused;
};

struct estimator_aid_source3d_s {
	uint64_t timestamp;
	uint64_t timestamp_sample;
	uint64_t time_last_fuse;
	uint32_t device_id;
	float observation[3];
	float observation_variance[3];
	float innovation[3];
	float innovation_variance[3];
	float test_ratio[3];
	uint8_t estimator_instance;
	bool fusion_enabled;
	bool innovation_rejected;
	bool fused;
	uint8_t _padding0[4]; // required for logger


};

#else

#include <mathlib/mathlib.h>

#endif  // ECL_STANDALONE
#endif  // MATHLIB_H
