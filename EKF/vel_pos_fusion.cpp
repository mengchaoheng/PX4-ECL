/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file vel_pos_fusion.cpp
 * Function for fusing gps and baro measurements/
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Siddharth Bharat Purohit <siddharthbharatpurohit@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "mathlib/mathlib.h"
#include "ekf.h"

bool Ekf::fuseHorizontalVelocity(const Vector3f &innov, const float innov_gate, const Vector3f &obs_var,
				 Vector3f &innov_var, Vector2f &test_ratio)
{
	innov_var(0) = P(4, 4) + obs_var(0);
	innov_var(1) = P(5, 5) + obs_var(1);
	test_ratio(0) = fmaxf(sq(innov(0)) / (sq(innov_gate) * innov_var(0)),
			      sq(innov(1)) / (sq(innov_gate) * innov_var(1)));

	const bool innov_check_pass = (test_ratio(0) <= 1.0f);

	if (innov_check_pass) {
		_innov_check_fail_status.flags.reject_hor_vel = false;

		bool fuse_vx = fuseVelPosHeight(innov(0), innov_var(0), 0);
		bool fuse_vy = fuseVelPosHeight(innov(1), innov_var(1), 1);

		return fuse_vx && fuse_vy;

	} else {
		_innov_check_fail_status.flags.reject_hor_vel = true;
		return false;
	}
}

bool Ekf::fuseVerticalVelocity(const Vector3f &innov, const float innov_gate, const Vector3f &obs_var,
			       Vector3f &innov_var, Vector2f &test_ratio)
{
	innov_var(2) = P(6, 6) + obs_var(2);
	test_ratio(1) = sq(innov(2)) / (sq(innov_gate) * innov_var(2));
	_vert_vel_innov_ratio = innov(2) / sqrtf(innov_var(2));
	_vert_vel_fuse_time_us = _time_last_imu;
	bool innov_check_pass = (test_ratio(1) <= 1.0f);

	// if there is bad vertical acceleration data, then don't reject measurement,
	// but limit innovation to prevent spikes that could destabilise the filter
	float innovation;

	if (_fault_status.flags.bad_acc_vertical && !innov_check_pass) {
		const float innov_limit = innov_gate * sqrtf(innov_var(2));
		innovation = math::constrain(innov(2), -innov_limit, innov_limit);
		innov_check_pass = true;

	} else {
		innovation = innov(2);
	}

	if (innov_check_pass) {
		_innov_check_fail_status.flags.reject_ver_vel = false;

		return fuseVelPosHeight(innovation, innov_var(2), 2);

	} else {
		_innov_check_fail_status.flags.reject_ver_vel = true;
		return false;
	}
}

bool Ekf::fuseHorizontalPosition(const Vector3f &innov, const float innov_gate, const Vector3f &obs_var,
				 Vector3f &innov_var, Vector2f &test_ratio, bool inhibit_gate)
{

	innov_var(0) = P(7, 7) + obs_var(0);
	innov_var(1) = P(8, 8) + obs_var(1);
	test_ratio(0) = fmaxf(sq(innov(0)) / (sq(innov_gate) * innov_var(0)),
			      sq(innov(1)) / (sq(innov_gate) * innov_var(1)));

	const bool innov_check_pass = test_ratio(0) <= 1.0f;

	if (innov_check_pass || inhibit_gate) {
		if (inhibit_gate && test_ratio(0) > sq(100.0f / innov_gate)) {
			// always protect against extreme values that could result in a NaN
			return false;
		}

		_innov_check_fail_status.flags.reject_hor_pos = false;

		bool fuse_x = fuseVelPosHeight(innov(0), innov_var(0), 3);
		bool fuse_y = fuseVelPosHeight(innov(1), innov_var(1), 4);

		return fuse_x && fuse_y;

	} else {
		_innov_check_fail_status.flags.reject_hor_pos = true;
		return false;
	}
}

bool Ekf::fuseVerticalPosition(const float innov, const float innov_gate, const float obs_var,
			       float &innov_var, float &test_ratio)
{
	innov_var = P(9, 9) + obs_var;
	test_ratio = sq(innov) / (sq(innov_gate) * innov_var);
	_vert_pos_innov_ratio = innov / sqrtf(innov_var);
	_vert_pos_fuse_attempt_time_us = _time_last_imu;
	bool innov_check_pass = test_ratio <= 1.0f;

	// if there is bad vertical acceleration data, then don't reject measurement,
	// but limit innovation to prevent spikes that could destabilise the filter
	float innovation;

	if (_fault_status.flags.bad_acc_vertical && !innov_check_pass) {
		const float innov_limit = innov_gate * sqrtf(innov_var);
		innovation = math::constrain(innov, -innov_limit, innov_limit);
		innov_check_pass = true;

	} else {
		innovation = innov;
	}

	if (innov_check_pass) {
		_innov_check_fail_status.flags.reject_ver_pos = false;

		return fuseVelPosHeight(innovation, innov_var, 5);

	} else {
		_innov_check_fail_status.flags.reject_ver_pos = true;
		return false;
	}
}

// Helper function that fuses a single velocity or position measurement
bool Ekf::fuseVelPosHeight(const float innov, const float innov_var, const int obs_index)
{
	Vector24f Kfusion;  // Kalman gain vector for any single observation - sequential fusion is used.
	const unsigned state_index = obs_index + 4;  // we start with vx and this is the 4. state

	// calculate kalman gain K = PHS, where S = 1/innovation variance
	for (int row = 0; row < _k_num_states; row++) {
		Kfusion(row) = P(row, state_index) / innov_var;
	}

	SquareMatrix24f KHP;

	for (unsigned row = 0; row < _k_num_states; row++) {
		for (unsigned column = 0; column < _k_num_states; column++) {
			KHP(row, column) = Kfusion(row) * P(state_index, column);
		}
	}

	// if the covariance correction will result in a negative variance, then
	// the covariance matrix is unhealthy and must be corrected
	bool healthy = true;

	for (int i = 0; i < _k_num_states; i++) {
		if (P(i, i) < KHP(i, i)) {
			// zero rows and columns
			P.uncorrelateCovarianceSetVariance<1>(i, 0.0f);

			healthy = false;
		}
	}

	setVelPosStatus(obs_index, healthy);

	if (healthy) {
		// apply the covariance corrections
		P -= KHP;

		fixCovarianceErrors(true);

		// apply the state corrections
		fuse(Kfusion, innov);

		return true;
	}

	return false;
}

void Ekf::setVelPosStatus(const int index, const bool healthy)
{
	switch (index) {
	case 0:
		if (healthy) {
			_fault_status.flags.bad_vel_N = false;
			_time_last_hor_vel_fuse = _time_last_imu;

		} else {
			_fault_status.flags.bad_vel_N = true;
		}

		break;

	case 1:
		if (healthy) {
			_fault_status.flags.bad_vel_E = false;
			_time_last_hor_vel_fuse = _time_last_imu;

		} else {
			_fault_status.flags.bad_vel_E = true;
		}

		break;

	case 2:
		if (healthy) {
			_fault_status.flags.bad_vel_D = false;
			_time_last_ver_vel_fuse = _time_last_imu;

		} else {
			_fault_status.flags.bad_vel_D = true;
		}

		break;

	case 3:
		if (healthy) {
			_fault_status.flags.bad_pos_N = false;
			_time_last_hor_pos_fuse = _time_last_imu;

		} else {
			_fault_status.flags.bad_pos_N = true;
		}

		break;

	case 4:
		if (healthy) {
			_fault_status.flags.bad_pos_E = false;
			_time_last_hor_pos_fuse = _time_last_imu;

		} else {
			_fault_status.flags.bad_pos_E = true;
		}

		break;

	case 5:
		if (healthy) {
			_fault_status.flags.bad_pos_D = false;
			_time_last_hgt_fuse = _time_last_imu;

		} else {
			_fault_status.flags.bad_pos_D = true;
		}

		break;
	}
}
