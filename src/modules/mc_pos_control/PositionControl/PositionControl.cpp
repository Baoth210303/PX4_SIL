/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>

using namespace matrix;

const trajectory_setpoint_s PositionControl::empty_trajectory_setpoint = {0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN};

void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void PositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
}

void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	// Given that the equation for thrust is T = a_sp * Th / g - Th
	// with a_sp = desired acceleration, Th = hover thrust and g = gravity constant,
	// we want to find the acceleration that needs to be added to the integrator in order obtain
	// the same thrust after replacing the current hover thrust by the new one.
	// T' = T => a_sp' * Th' / g - Th' = a_sp * Th / g - Th
	// so a_sp' = (a_sp - g) * Th / Th' + g
	// we can then add a_sp' - a_sp to the current integrator to absorb the effect of changing Th by Th'
	const float previous_hover_thrust = _hover_thrust;
	setHoverThrust(hover_thrust_new);

	if (PX4_ISFINITE(_acc_sp(2))) {
		_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
			       + CONSTANTS_ONE_G - _acc_sp(2);
	}

}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void PositionControl::setInputSetpoint(const trajectory_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.position);
	_vel_sp = Vector3f(setpoint.velocity);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

bool PositionControl::update(const float dt)
{
	bool valid = _inputValid();

	if (valid) {
		_smcPositionVelocityControl(dt);

		_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
		_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control
	}

	// There has to be a valid output acceleration and thrust setpoint otherwise something went wrong
	return valid && _acc_sp.isAllFinite() && _thr_sp.isAllFinite();
}

void PositionControl::_positionControl()
{
	// P-position controller
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

void PositionControl::_velocityControl(const float dt)
{
	// Constrain vertical velocity integral
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

	// PID velocity control
	Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

	// No control input from setpoints or corresponding states which are NAN
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	_accelerationControl();

	// Integrator anti-windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.f)) {
		vel_error(2) = 0.f;
	}

	// Prioritize vertical control while keeping a horizontal margin
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	// Determine how much vertical thrust is left keeping horizontal margin
	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// Saturate maximal vertical thrust
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// Determine how much horizontal thrust is left after prioritizing vertical control
	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0.f;

	if (thrust_max_xy_squared > 0.f) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in horizontal direction
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	const Vector2f acc_sp_xy_produced = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);

	// The produced acceleration can be greater or smaller than the desired acceleration due to the saturations and the actual vertical thrust (computed independently).
	// The ARW loop needs to run if the signal is saturated only.
	if (_acc_sp.xy().norm_squared() > acc_sp_xy_produced.norm_squared()) {
		const float arw_gain = 2.f / _gain_vel_p(0);
		const Vector2f acc_sp_xy = _acc_sp.xy();

		vel_error.xy() = Vector2f(vel_error) - arw_gain * (acc_sp_xy - acc_sp_xy_produced);
	}

	// Make sure integral doesn't get NAN
	ControlMath::setZeroIfNanVector3f(vel_error);
	// Update integral part of velocity control
	_vel_int += vel_error.emult(_gain_vel_i) * dt;
}

void PositionControl::_smcPositionVelocityControl(const float dt)
{
	/*
	 * Preserve PX4 setpoint semantics:
	 *
	 * - finite position setpoint    -> position tracking
	 * - finite velocity setpoint    -> velocity tracking / feed-forward
	 * - finite acceleration setpoint -> acceleration feed-forward
	 *
	 * NAN means that particular setpoint component is not commanded.
	 */
	Vector3f e{};
	Vector3f e_dot{};
	Vector3f acc_ff = _acc_sp;

	for (int i = 0; i < 3; i++) {
		const bool has_pos_sp = PX4_ISFINITE(_pos_sp(i));
		const bool has_vel_sp = PX4_ISFINITE(_vel_sp(i));

		if (has_pos_sp) {
			e(i) = _pos(i) - _pos_sp(i);

			// If no velocity feed-forward is supplied, track zero
			// velocity while converging to the position setpoint.
			const float vel_sp = has_vel_sp ? _vel_sp(i) : 0.f;
			e_dot(i) = _vel(i) - vel_sp;

		} else if (has_vel_sp) {
			// Velocity-only control: no position-error contribution.
			e(i) = 0.f;
			e_dot(i) = _vel(i) - _vel_sp(i);

		} else {
			// Acceleration-only feed-forward: no feedback error terms.
			e(i) = 0.f;
			e_dot(i) = 0.f;
		}

		if (!PX4_ISFINITE(acc_ff(i))) {
			acc_ff(i) = 0.f;
		}
	}

	/*
	 * Reset analytical-reference status for this sample.
	 * The reference becomes valid again only after all required
	 * quantities have been computed successfully.
	 */
	_smc_position_reference_valid = false;
	_smc_position_reference_saturated = false;
	_kappa.zero();
	_kappa_dot.zero();
	_kappa_ddot.zero();

	/*
	 * Derivative timing.
	 */
	const bool derivative_dt_valid =
		PX4_ISFINITE(dt) && dt > 1e-4f && dt < 0.1f;

	/*
	 * Analytical derivatives are considered valid only when all
	 * required histories already existed before this control sample.
	 */
	const bool derivative_history_valid =
		derivative_dt_valid
		&& _smc_acc_ff_history_initialized
		&& _smc_jerk_des_history_initialized
		&& _smc_acc_meas_history_initialized;

	/*
	 * Desired acceleration derivatives.
	 *
	 * Keep the same convention as the public implementation used
	 * previously for comparison with the paper:
	 *
	 * jerk_d = (a_d[k] - a_d[k-1]) * dt
	 * snap_d = (jerk_d[k] - jerk_d[k-1]) * dt
	 *
	 * This intentionally is not the conventional finite difference /dt.
	 */
	Vector3f jerk_des{};

	if (_smc_acc_ff_history_initialized && derivative_dt_valid) {
		jerk_des = (acc_ff - _smc_acc_ff_prev) * dt;
	}

	_smc_acc_ff_prev = acc_ff;
	_smc_acc_ff_history_initialized = true;

	Vector3f snap_des{};

	if (_smc_jerk_des_history_initialized && derivative_dt_valid) {
		snap_des = (jerk_des - _smc_jerk_des_prev) * dt;
	}

	_smc_jerk_des_prev = jerk_des;
	_smc_jerk_des_history_initialized = true;

	/*
	 * Current translational acceleration and jerk.
	 * _vel_dot is PX4's velocity derivative estimate.
	 */
	Vector3f acc_meas = _vel_dot;
	ControlMath::setZeroIfNanVector3f(acc_meas);

	Vector3f jerk_meas{};

	if (_smc_acc_meas_history_initialized && derivative_dt_valid) {
		jerk_meas = (acc_meas - _smc_acc_meas_prev) * dt;
	}

	_smc_acc_meas_prev = acc_meas;
	_smc_acc_meas_history_initialized = true;

	/*
	 * Translational SMC:
	 *
	 * e     = p - p_d
	 * e_dot = v - v_d
	 * s     = e_dot + Lambda e
	 *
	 * a_cmd = a_d
	 *         - Lambda e_dot
	 *         - K s
	 *         - eta tanh(rho s)
	 */
	const Vector3f sliding_surface = e_dot + e.emult(_smc_lambda);

	Vector3f switching_term{};
	Vector3f sech2{};

	for (int i = 0; i < 3; i++) {
		const float z = _smc_rho(i) * sliding_surface(i);
		switching_term(i) = tanhf(z);
		sech2(i) = 1.f - switching_term(i) * switching_term(i);
	}

	/*
	 * Acceleration/jerk tracking errors and sliding-variable derivatives.
	 */
	const Vector3f e_acc = acc_meas - acc_ff;
	const Vector3f e_jerk = jerk_meas - jerk_des;

	const Vector3f sliding_surface_dot =
		e_acc + e_dot.emult(_smc_lambda);

	const Vector3f sliding_surface_ddot =
		e_jerk + e_acc.emult(_smc_lambda);

	/*
	 * Derivatives of phi = tanh(rho * s).
	 */
	Vector3f switching_term_dot{};
	Vector3f switching_term_ddot{};

	for (int i = 0; i < 3; i++) {
		const float rho = _smc_rho(i);

		switching_term_dot(i) =
			rho * sech2(i) * sliding_surface_dot(i);

		switching_term_ddot(i) =
			rho * sech2(i) * sliding_surface_ddot(i)
			- 2.f * rho * rho
			* sech2(i) * switching_term(i)
			* sliding_surface_dot(i) * sliding_surface_dot(i);
	}

	/*
	 * Position-SMC acceleration command and its analytical derivatives.
	 */
	const Vector3f acc_cmd =
		acc_ff
		- e_dot.emult(_smc_lambda)
		- sliding_surface.emult(_smc_k)
		- switching_term.emult(_smc_eta);

	const Vector3f acc_cmd_dot =
		jerk_des
		- e_acc.emult(_smc_lambda)
		- sliding_surface_dot.emult(_smc_k)
		- switching_term_dot.emult(_smc_eta);

	const Vector3f acc_cmd_ddot =
		snap_des
		- e_jerk.emult(_smc_lambda)
		- sliding_surface_ddot.emult(_smc_k)
		- switching_term_ddot.emult(_smc_eta);

	_acc_sp = acc_cmd;

	/*
	 * Force-direction reference for the attitude layer.
	 *
	 * This is still translational information only. The attitude layer
	 * will later construct R_d, q_d, omega_Rd and alpha_Rd.
	 */
	_kappa = Vector3f(
		-acc_cmd(0),
		-acc_cmd(1),
		CONSTANTS_ONE_G);

	_kappa_dot = Vector3f(
		-acc_cmd_dot(0),
		-acc_cmd_dot(1),
		0.f);

	_kappa_ddot = Vector3f(
		-acc_cmd_ddot(0),
		-acc_cmd_ddot(1),
		0.f);

	if (!_decouple_horizontal_and_vertical_acceleration) {
		_kappa(2) = CONSTANTS_ONE_G - acc_cmd(2);
		_kappa_dot(2) = -acc_cmd_dot(2);
		_kappa_ddot(2) = -acc_cmd_ddot(2);
	}

	/*
	 * Analytical derivatives cannot be used until the derivative
	 * histories are initialized and all quantities are finite.
	 */
	if (!derivative_history_valid
	    || !_kappa.isAllFinite()
	    || !_kappa_dot.isAllFinite()
	    || !_kappa_ddot.isAllFinite()) {

		_kappa_dot.zero();
		_kappa_ddot.zero();
	}

	/*
	 * Detect whether PX4's tilt limiter will modify the desired
	 * thrust direction. If it does, the analytical derivatives no
	 * longer represent the attitude setpoint actually sent downstream.
	 */
	const float kappa_norm = _kappa.norm();

	if (!PX4_ISFINITE(kappa_norm) || kappa_norm <= FLT_EPSILON) {
		_kappa.zero();
		_kappa_dot.zero();
		_kappa_ddot.zero();

	} else {
		const Vector3f body_z_unlimited = _kappa / kappa_norm;

		const float cos_tilt = math::constrain(
			body_z_unlimited.dot(Vector3f(0.f, 0.f, 1.f)),
			-1.f,
			1.f);

		const float tilt = acosf(cos_tilt);

		if (PX4_ISFINITE(_lim_tilt)
		    && _lim_tilt > 0.f
		    && tilt > (_lim_tilt + 1e-4f)) {

			_smc_position_reference_saturated = true;
			_kappa_dot.zero();
			_kappa_ddot.zero();
		}
	}

	/*
	 * Keep PX4's original acceleration-to-thrust conversion.
	 * This preserves tilt limiting, hover-thrust scaling and the
	 * existing thrust-vector convention.
	 */
	_accelerationControl();

	/*
	 * Keep PX4's original thrust saturation strategy.
	 * PID anti-windup is deliberately not copied because the SMC
	 * controller has no velocity integrator.
	 */
	const Vector3f thrust_before_saturation = _thr_sp;

	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	const float allocated_horizontal_thrust =
		math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);

	const float thrust_z_max_squared =
		thrust_max_squared - math::sq(allocated_horizontal_thrust);

	_thr_sp(2) =
		math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	const float thrust_max_xy_squared =
		thrust_max_squared - math::sq(_thr_sp(2));

	float thrust_max_xy = 0.f;

	if (thrust_max_xy_squared > 0.f) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	if (thrust_sp_xy_norm > thrust_max_xy
	    && thrust_sp_xy_norm > FLT_EPSILON) {
		_thr_sp.xy() =
			thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	/*
	 * If thrust saturation changed the vector sent downstream,
	 * the analytical kappa derivatives no longer describe exactly
	 * the attitude setpoint generated from the final thrust vector.
	 */
	if ((_thr_sp - thrust_before_saturation).norm() > 1e-5f) {
		_smc_position_reference_saturated = true;
		_kappa_dot.zero();
		_kappa_ddot.zero();
	}

	/*
	 * Publishable analytical reference is valid only when:
	 *  - derivative histories are established,
	 *  - all reference quantities are finite and non-degenerate,
	 *  - no PX4 tilt/thrust saturation modified the commanded direction.
	 */
	const float final_kappa_norm = _kappa.norm();

	_smc_position_reference_valid =
		derivative_history_valid
		&& _kappa.isAllFinite()
		&& _kappa_dot.isAllFinite()
		&& _kappa_ddot.isAllFinite()
		&& PX4_ISFINITE(final_kappa_norm)
		&& final_kappa_norm > FLT_EPSILON
		&& !_smc_position_reference_saturated;

	if (!_smc_position_reference_valid) {
		_kappa_dot.zero();
		_kappa_ddot.zero();
	}
}

void PositionControl::_accelerationControl()
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	float z_specific_force = -CONSTANTS_ONE_G;

	if (!_decouple_horizontal_and_vertical_acceleration) {
		// Include vertical acceleration setpoint for better horizontal acceleration tracking
		z_specific_force += _acc_sp(2);
	}

	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), -z_specific_force).normalized();
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	// Convert to thrust assuming hover thrust produces standard gravity
	const float thrust_ned_z = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// Project thrust to planned body attitude
	const float cos_ned_body = (Vector3f(0, 0, 1).dot(body_z));
	const float collective_thrust = math::min(thrust_ned_z / cos_ned_body, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust;
}

bool PositionControl::_inputValid()
{
	bool valid = true;

	// Every axis x, y, z needs to have some setpoint
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	// x and y input setpoints always have to come in pairs
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
	valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			// Position SMC uses both position and velocity feedback.
			valid = valid && PX4_ISFINITE(_pos(i)) && PX4_ISFINITE(_vel(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
