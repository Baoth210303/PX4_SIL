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
 * 2. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 ****************************************************************************/

/**
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"

#include <float.h>

#include <drivers/drv_hrt.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>


using namespace matrix;


namespace
{


Matrix3f skewMatrix(
	const Vector3f &v)
{
	Matrix3f S{};

	S(0, 1) =
		-v(2);

	S(0, 2) =
		v(1);

	S(1, 0) =
		v(2);

	S(1, 2) =
		-v(0);

	S(2, 0) =
		-v(1);

	S(2, 1) =
		v(0);

	return S;
}


/**
 * Extract vee operator using only the
 * skew-symmetric part.
 *
 * This suppresses small numerical
 * symmetric errors.
 */
Vector3f veeSkew(
	const Matrix3f &M)
{
	return Vector3f(

		0.5f
		* (M(2, 1) - M(1, 2)),

		0.5f
		* (M(0, 2) - M(2, 0)),

		0.5f
		* (M(1, 0) - M(0, 1))
	);
}


/**
 * For:
 *
 * n = v / ||v||
 *
 * calculate:
 *
 * n
 * n_dot
 * n_ddot
 *
 * analytically.
 */
bool normalizedVectorDerivatives(
	const Vector3f &v,
	const Vector3f &v_dot,
	const Vector3f &v_ddot,
	Vector3f &n,
	Vector3f &n_dot,
	Vector3f &n_ddot)
{
	const float r =
		v.norm();


	if (!PX4_ISFINITE(r)
	    || r < 1e-5f) {

		n.zero();

		n_dot.zero();

		n_ddot.zero();

		return false;
	}


	n =
		v / r;


	const float r_dot =
		n.dot(
			v_dot);


	/**
	 * n_dot =
	 *
	 * [v_dot - n(n^T v_dot)] / ||v||
	 */
	n_dot =

		(v_dot
		 - n * r_dot)

		/ r;


	/**
	 * n_ddot =
	 *
	 * P*v_ddot/r
	 *
	 * -2*(r_dot/r)*n_dot
	 *
	 * -n*||n_dot||^2
	 */
	n_ddot =

		(v_ddot
		 - n * n.dot(v_ddot))

		/ r

		- 2.f
		* (r_dot / r)
		* n_dot

		- n
		* n_dot.norm_squared();


	return n.isAllFinite()
	       && n_dot.isAllFinite()
	       && n_ddot.isAllFinite();
}


bool vectorsDifferent(
	const Vector3f &a,
	const Vector3f &b,
	float eps = 1e-5f)
{
	return (a - b).norm()
	       > eps;
}


} // namespace


const trajectory_setpoint_s
PositionControl::empty_trajectory_setpoint = {

	0,

	{NAN, NAN, NAN},

	{NAN, NAN, NAN},

	{NAN, NAN, NAN},

	{NAN, NAN, NAN},

	NAN,

	NAN
};


void
PositionControl::setVelocityGains(
	const Vector3f &P,
	const Vector3f &I,
	const Vector3f &D)
{
	_gain_vel_p =
		P;

	_gain_vel_i =
		I;

	_gain_vel_d =
		D;
}


void
PositionControl::setVelocityLimits(
	const float vel_horizontal,
	const float vel_up,
	const float vel_down)
{
	_lim_vel_horizontal =
		vel_horizontal;

	_lim_vel_up =
		vel_up;

	_lim_vel_down =
		vel_down;
}


void
PositionControl::setThrustLimits(
	const float min,
	const float max)
{
	_lim_thr_min =
		math::max(
			min,
			10e-4f);

	_lim_thr_max =
		max;
}


void
PositionControl::setHorizontalThrustMargin(
	const float margin)
{
	_lim_thr_xy_margin =
		margin;
}


void
PositionControl::updateHoverThrust(
	const float hover_thrust_new)
{
	const float previous_hover_thrust =
		_hover_thrust;


	setHoverThrust(
		hover_thrust_new);


	if (PX4_ISFINITE(
		    _acc_sp(2))) {

		_vel_int(2) +=

			(_acc_sp(2)
			 - CONSTANTS_ONE_G)

			* previous_hover_thrust

			/ _hover_thrust

			+ CONSTANTS_ONE_G

			- _acc_sp(2);
	}
}


void
PositionControl::setState(
	const PositionControlStates &states)
{
	_pos =
		states.position;


	_vel =
		states.velocity;


	_yaw =
		states.yaw;


	_vel_dot =
		states.acceleration;
}


void
PositionControl::setInputSetpoint(
	const trajectory_setpoint_s &setpoint)
{
	_pos_sp =
		Vector3f(
			setpoint.position);


	_vel_sp =
		Vector3f(
			setpoint.velocity);


	_acc_sp =
		Vector3f(
			setpoint.acceleration);


	_jerk_sp =
		Vector3f(
			setpoint.jerk);


	_yaw_sp =
		setpoint.yaw;


	_yawspeed_sp =
		setpoint.yawspeed;
}


bool
PositionControl::update(
	const float dt)
{
	bool valid =
		_inputValid();


	if (valid) {

		/**
		 * The analytical attitude-reference generator
		 * needs finite yaw and yaw-rate values.
		 */
		_yawspeed_sp =

			PX4_ISFINITE(
				_yawspeed_sp)

			? _yawspeed_sp

			: 0.f;


		_yaw_sp =

			PX4_ISFINITE(
				_yaw_sp)

			? _yaw_sp

			: _yaw;


		/**
		 * Full SMC replacement of original PX4
		 * P-position + PID-velocity cascade.
		 */
		_smcPositionVelocityControl(
			dt);
	}


	return valid
	       && _acc_sp.isAllFinite()
	       && _thr_sp.isAllFinite();
}


void
PositionControl::_smcPositionVelocityControl(
	const float dt)
{
	// =========================================================
	// Reset attitude-reference status for current sample
	// =========================================================

	_attitude_reference_valid =
		false;


	_attitude_reference_saturated =
		false;


	_omega_Rd.zero();

	_omega_dot_Rd.zero();


	// =========================================================
	// Local copies of trajectory setpoints
	// =========================================================

	Vector3f pos_sp =
		_pos_sp;


	Vector3f vel_sp =
		_vel_sp;


	Vector3f acc_ff =
		_acc_sp;


	Vector3f jerk_des{};


	// =========================================================
	// Preserve PX4 no-thrust convention
	// =========================================================

	const bool px4_no_thrust_request =

		PX4_ISFINITE(
			acc_ff(2))

		&& (acc_ff(2)
		    > 2.f
		    * CONSTANTS_ONE_G);


	if (px4_no_thrust_request) {

		_acc_sp =
			Vector3f(
				0.f,
				0.f,
				CONSTANTS_ONE_G);


		_thr_sp.zero();


		_kappa =
			Vector3f(
				0.f,
				0.f,
				1.f);


		_kappa_dot.zero();

		_kappa_ddot.zero();


		_attitude_reference_saturated =
			true;


		return;
	}


	// =========================================================
	// Near-ground landing handling
	// =========================================================

	const bool smc_landing_descent =

		PX4_ISFINITE(
			_vel_sp(2))

		&& (_vel_sp(2)
		    > 0.1f);


	const bool smc_near_ground =

		PX4_ISFINITE(
			_pos(2))

		&& (_pos(2)
		    >
		    -_smc_landing_ground_threshold);


	const bool smc_force_zero_thrust =

		smc_landing_descent

		&& smc_near_ground;


	// =========================================================
	// Sanitize setpoints
	// =========================================================

	for (int i = 0;
	     i < 3;
	     i++) {

		const bool has_pos_sp =
			PX4_ISFINITE(
				_pos_sp(i));


		const bool has_vel_sp =
			PX4_ISFINITE(
				_vel_sp(i));


		if (!has_pos_sp) {

			pos_sp(i) =

				PX4_ISFINITE(
					_pos(i))

				? _pos(i)

				: 0.f;
		}


		if (!has_vel_sp) {

			vel_sp(i) =

				has_pos_sp

				? 0.f

				:
				(
					PX4_ISFINITE(
						_vel(i))

					? _vel(i)

					: 0.f
				);
		}


		if (!PX4_ISFINITE(
			    acc_ff(i))) {

			acc_ff(i) =
				0.f;
		}

	}


	// =========================================================
	// Derivative timing
	// =========================================================

	const bool derivative_dt_valid =

		PX4_ISFINITE(
			dt)

		&& dt > 1e-4f

		&& dt < 0.1f;


	// =========================================================
	// Public CrazyAQSMC-style desired derivatives
	//
	// The public implementation does not use the trajectory
	// jerk setpoint directly. Instead it computes:
	//
	// jerk_d =
	// dt * (acc_d[k] - acc_d[k-1])
	//
	// snap_d =
	// dt * (jerk_d[k] - jerk_d[k-1])
	//
	// NOTE:
	// This intentionally reproduces the public paper code
	// implementation for comparison. It is not the standard
	// finite-difference derivative (Delta x / dt).
	// =========================================================

	static Vector3f acc_des_prev{};

	static bool acc_des_history_initialized =
		false;


	if (acc_des_history_initialized
	    && derivative_dt_valid) {

		jerk_des =

			(acc_ff
			 - acc_des_prev)

			* dt;
	}


	acc_des_prev =
		acc_ff;


	acc_des_history_initialized =
		true;


	Vector3f snap_des{};


	if (_jerk_sp_history_initialized
	    && derivative_dt_valid) {

		snap_des =

			(jerk_des
			 - _jerk_sp_prev)

			* dt;
	}


	_jerk_sp_prev =
		jerk_des;


	_jerk_sp_history_initialized =
		true;


	// =========================================================
	// Current translational jerk
	//
	// Current acceleration is already the filtered PX4
	// velocity derivative.
	//
	// Public CrazyAQSMC-style implementation:
	//
	// jerk =
	// dt * (acc[k] - acc[k-1])
	//
	// This intentionally reproduces the public implementation.
	// =========================================================

	Vector3f acc_meas =
		_vel_dot;


	ControlMath::setZeroIfNanVector3f(
		acc_meas);


	Vector3f jerk_meas{};


	if (_acc_meas_history_initialized
	    && derivative_dt_valid) {

		jerk_meas =

			(acc_meas
			 - _acc_meas_prev)

			* dt;
	}


	_acc_meas_prev =
		acc_meas;


	_acc_meas_history_initialized =
		true;


	// =========================================================
	// Position SMC
	//
	// e     = p - p_d
	//
	// e_dot = v - v_d
	//
	// s     = e_dot + Lambda e
	// =========================================================

	const Vector3f e =

		_pos
		- pos_sp;


	const Vector3f e_dot =

		_vel
		- vel_sp;


	const Vector3f s =

		e_dot

		+ e.emult(
			_smc_lambda);


	// =========================================================
	// Smooth switching term
	//
	// phi =
	//
	// tanh(rho s)
	//
	// This replaces the old hard sat(s) in Position SMC.
	// =========================================================

	Vector3f phi{};

	Vector3f sech2{};


	for (int i = 0;
	     i < 3;
	     i++) {

		const float z =

			_smc_rho(i)
			* s(i);


		phi(i) =
			tanhf(z);


		/**
		 * sech^2(z)
		 *
		 * = 1 - tanh^2(z)
		 */
		sech2(i) =

			1.f

			- phi(i)
			* phi(i);
	}


	// =========================================================
	// Acceleration and jerk errors
	// =========================================================

	const Vector3f e_acc =

		acc_meas

		- acc_ff;


	const Vector3f e_jerk =

		jerk_meas

		- jerk_des;


	// =========================================================
	// Sliding-variable derivatives
	//
	// s_dot =
	//
	// e_acc + Lambda e_dot
	//
	//
	// s_ddot =
	//
	// e_jerk + Lambda e_acc
	// =========================================================

	const Vector3f s_dot =

		e_acc

		+ e_dot.emult(
			_smc_lambda);


	const Vector3f s_ddot =

		e_jerk

		+ e_acc.emult(
			_smc_lambda);


	// =========================================================
	// Derivatives of tanh(rho s)
	// =========================================================

	Vector3f phi_dot{};

	Vector3f phi_ddot{};


	for (int i = 0;
	     i < 3;
	     i++) {

		const float rho =
			_smc_rho(i);


		/**
		 * phi_dot =
		 *
		 * rho sech^2(rho s) s_dot
		 */
		phi_dot(i) =

			rho

			* sech2(i)

			* s_dot(i);


		/**
		 * phi_ddot =
		 *
		 * rho sech^2(rho s) s_ddot
		 *
		 * -2 rho^2
		 * sech^2(rho s)
		 * tanh(rho s)
		 * s_dot^2
		 */
		phi_ddot(i) =

			rho
			* sech2(i)
			* s_ddot(i)

			- 2.f
			* rho
			* rho
			* sech2(i)
			* phi(i)
			* s_dot(i)
			* s_dot(i);
	}


	// =========================================================
	// Smooth Position SMC law
	//
	// a_c =
	//
	// a_d
	//
	// - Lambda e_dot
	//
	// - K s
	//
	// - eta tanh(rho s)
	// =========================================================

	const Vector3f acc_cmd_nominal =

		acc_ff

		- e_dot.emult(
			_smc_lambda)

		- s.emult(
			_smc_k)

		- phi.emult(
			_smc_eta);


	// =========================================================
	// Analytical derivative of acceleration command
	//
	// a_c_dot =
	//
	// jerk_d
	//
	// - Lambda e_acc
	//
	// - K s_dot
	//
	// - eta phi_dot
	// =========================================================

	const Vector3f acc_cmd_dot =

		jerk_des

		- e_acc.emult(
			_smc_lambda)

		- s_dot.emult(
			_smc_k)

		- phi_dot.emult(
			_smc_eta);


	// =========================================================
	// Second derivative
	//
	// a_c_ddot =
	//
	// snap_d
	//
	// - Lambda e_jerk
	//
	// - K s_ddot
	//
	// - eta phi_ddot
	// =========================================================

	const Vector3f acc_cmd_ddot =

		snap_des

		- e_jerk.emult(
			_smc_lambda)

		- s_ddot.emult(
			_smc_k)

		- phi_ddot.emult(
			_smc_eta);


	Vector3f acc_cmd =
		acc_cmd_nominal;


	ControlMath::setZeroIfNanVector3f(
		acc_cmd);


	// =========================================================
	// Existing PX4/SMC acceleration safety limits
	//
	// Hard saturation is deliberately kept for safety.
	//
	// If it becomes active, analytical derivatives are no
	// longer exact and feed-forward omega/alpha is disabled
	// for that sample.
	// =========================================================

	const Vector3f acc_before_limits =
		acc_cmd;


	const float acc_xy_norm =
		acc_cmd.xy().norm();


	if (acc_xy_norm
	    > _smc_acc_xy_max

	    && acc_xy_norm
	    > FLT_EPSILON) {

		acc_cmd.xy() =

			acc_cmd.xy()

			/ acc_xy_norm

			* _smc_acc_xy_max;
	}


	acc_cmd(2) =

		math::constrain(

			acc_cmd(2),

			-_smc_acc_z_max,

			_smc_acc_z_max);


	if (vectorsDifferent(
		    acc_before_limits,
		    acc_cmd)) {

		_attitude_reference_saturated =
			true;
	}


	// =========================================================
	// Landing handling
	// =========================================================

	if (smc_force_zero_thrust) {

		acc_cmd =
			Vector3f(
				0.f,
				0.f,
				CONSTANTS_ONE_G);


		_attitude_reference_saturated =
			true;
	}


	_acc_sp =
		acc_cmd;


	// =========================================================
	// Paper-style force direction kappa
	//
	// PX4 NED desired body-z before tilt limiting:
	//
	// decoupled:
	//
	// [-a_x, -a_y, g]
	//
	// coupled:
	//
	// [-a_x, -a_y, g-a_z]
	//
	// A constant mass scale is omitted.
	// =========================================================

	_kappa =

		Vector3f(

			-acc_cmd(0),

			-acc_cmd(1),

			CONSTANTS_ONE_G);


	_kappa_dot =

		Vector3f(

			-acc_cmd_dot(0),

			-acc_cmd_dot(1),

			0.f);


	_kappa_ddot =

		Vector3f(

			-acc_cmd_ddot(0),

			-acc_cmd_ddot(1),

			0.f);


	if (!_decouple_horizontal_and_vertical_acceleration) {

		_kappa(2) =

			CONSTANTS_ONE_G

			- acc_cmd(2);


		_kappa_dot(2) =

			-acc_cmd_dot(2);


		_kappa_ddot(2) =

			-acc_cmd_ddot(2);
	}


	if (_attitude_reference_saturated) {

		_kappa_dot.zero();

		_kappa_ddot.zero();
	}


	// =========================================================
	// Original PX4 acceleration -> thrust conversion
	// =========================================================

	_accelerationControl();


	if (smc_force_zero_thrust) {

		_thr_sp.zero();

		_omega_Rd.zero();

		_omega_dot_Rd.zero();

		return;
	}


	// =========================================================
	// Original PX4 thrust saturation strategy
	// =========================================================

	const Vector3f thrust_before_limits =
		_thr_sp;


	const Vector2f thrust_sp_xy(
		_thr_sp);


	const float thrust_sp_xy_norm =
		thrust_sp_xy.norm();


	const float thrust_max_squared =
		math::sq(
			_lim_thr_max);


	const float allocated_horizontal_thrust =

		math::min(

			thrust_sp_xy_norm,

			_lim_thr_xy_margin);


	const float thrust_z_max_squared =

		math::max(

			0.f,

			thrust_max_squared

			- math::sq(
				allocated_horizontal_thrust));


	_thr_sp(2) =

		math::max(

			_thr_sp(2),

			-sqrtf(
				thrust_z_max_squared));


	const float thrust_max_xy_squared =

		math::max(

			0.f,

			thrust_max_squared

			- math::sq(
				_thr_sp(2)));


	const float thrust_max_xy =
		sqrtf(
			thrust_max_xy_squared);


	if (thrust_sp_xy_norm
	    > thrust_max_xy

	    && thrust_sp_xy_norm
	    > FLT_EPSILON) {

		_thr_sp.xy() =

			thrust_sp_xy

			/ thrust_sp_xy_norm

			* thrust_max_xy;
	}


	if (vectorsDifferent(
		    thrust_before_limits,
		    _thr_sp)) {

		_attitude_reference_saturated =
			true;


		_kappa_dot.zero();

		_kappa_ddot.zero();
	}


	// =========================================================
	// Analytical desired attitude trajectory
	// =========================================================

	_updateAttitudeReference();
}


void
PositionControl::_updateAttitudeReference()
{
	_attitude_reference_valid =
		false;


	_omega_Rd.zero();

	_omega_dot_Rd.zero();


	if (!_kappa.isAllFinite()
	    || !_kappa_dot.isAllFinite()
	    || !_kappa_ddot.isAllFinite()) {

		return;
	}


	// =========================================================
	// b3d = kappa / ||kappa||
	// =========================================================

	Vector3f b3d{};

	Vector3f b3d_dot{};

	Vector3f b3d_ddot{};


	if (!normalizedVectorDerivatives(

		    _kappa,

		    _kappa_dot,

		    _kappa_ddot,

		    b3d,

		    b3d_dot,

		    b3d_ddot)) {

		return;
	}


	// =========================================================
	// Check whether PX4 tilt limiter would be active
	// =========================================================

	const float cos_tilt =

		math::constrain(

			b3d.dot(
				Vector3f(
					0.f,
					0.f,
					1.f)),

			-1.f,

			1.f);


	const float tilt =
		acosf(
			cos_tilt);


	if (PX4_ISFINITE(
		    _lim_tilt)

	    && _lim_tilt > 0.f

	    && tilt
	    > (_lim_tilt + 1e-4f)) {

		_attitude_reference_saturated =
			true;

		return;
	}


	if (_attitude_reference_saturated) {

		return;
	}


	// =========================================================
	// Desired yaw trajectory
	// =========================================================

	const float psi =
		_yaw_sp;


	const float psi_dot =
		_yawspeed_sp;


	if (!PX4_ISFINITE(
		    psi)

	    || !PX4_ISFINITE(
		    psi_dot)) {

		return;
	}


	// =========================================================
	// Heading vector
	//
	// b1r =
	//
	// [cos psi
	//  sin psi
	//  0]
	// =========================================================

	const Vector3f b1r(

		cosf(psi),

		sinf(psi),

		0.f);


	const Vector3f b1r_dot(

		-sinf(psi)
		* psi_dot,

		cosf(psi)
		* psi_dot,

		0.f);


	/**
	 * Same assumption as public CrazyAQSMC code:
	 *
	 * yaw angular acceleration = 0.
	 */
	const Vector3f b1r_ddot(

		-cosf(psi)
		* psi_dot
		* psi_dot,

		-sinf(psi)
		* psi_dot
		* psi_dot,

		0.f);


	// =========================================================
	// b2d =
	//
	// normalize(b3d x b1r)
	// =========================================================

	const Vector3f chi =

		b3d
		% b1r;


	const Vector3f chi_dot =

		b3d_dot
		% b1r

		+ b3d
		% b1r_dot;


	const Vector3f chi_ddot =

		b3d_ddot
		% b1r

		+ 2.f
		* (b3d_dot
		   % b1r_dot)

		+ b3d
		% b1r_ddot;


	Vector3f b2d{};

	Vector3f b2d_dot{};

	Vector3f b2d_ddot{};


	if (!normalizedVectorDerivatives(

		    chi,

		    chi_dot,

		    chi_ddot,

		    b2d,

		    b2d_dot,

		    b2d_ddot)) {

		return;
	}


	// =========================================================
	// b1d = b2d x b3d
	// =========================================================

	const Vector3f b1d =

		b2d
		% b3d;


	const Vector3f b1d_dot =

		b2d_dot
		% b3d

		+ b2d
		% b3d_dot;


	const Vector3f b1d_ddot =

		b2d_ddot
		% b3d

		+ 2.f
		* (b2d_dot
		   % b3d_dot)

		+ b2d
		% b3d_ddot;


	// =========================================================
	// Construct:
	//
	// R_d
	// R_d_dot
	// R_d_ddot
	// =========================================================

	Matrix3f R_dot{};

	Matrix3f R_ddot{};


	for (int i = 0;
	     i < 3;
	     i++) {

		_R_d(i, 0) =
			b1d(i);

		_R_d(i, 1) =
			b2d(i);

		_R_d(i, 2) =
			b3d(i);


		R_dot(i, 0) =
			b1d_dot(i);

		R_dot(i, 1) =
			b2d_dot(i);

		R_dot(i, 2) =
			b3d_dot(i);


		R_ddot(i, 0) =
			b1d_ddot(i);

		R_ddot(i, 1) =
			b2d_ddot(i);

		R_ddot(i, 2) =
			b3d_ddot(i);
	}


	// =========================================================
	// Desired angular velocity
	//
	// omega_Rd =
	//
	// (R_d^T R_d_dot)^vee
	// =========================================================

	const Matrix3f omega_hat =

		_R_d.transpose()

		* R_dot;


	_omega_Rd =

		veeSkew(
			omega_hat);


	// =========================================================
	// Desired angular acceleration
	//
	// alpha_Rd =
	//
	// [R_d^T R_d_ddot
	//
	// -(omega_Rd^x)^2]^vee
	// =========================================================

	const Matrix3f omega_cross =

		skewMatrix(
			_omega_Rd);


	const Matrix3f alpha_hat =

		_R_d.transpose()

		* R_ddot

		- omega_cross
		* omega_cross;


	_omega_dot_Rd =

		veeSkew(
			alpha_hat);


	if (!_omega_Rd.isAllFinite()
	    || !_omega_dot_Rd.isAllFinite()) {

		_omega_Rd.zero();

		_omega_dot_Rd.zero();

		return;
	}


	_attitude_reference_valid =
		true;
}


void
PositionControl::_positionControl()
{
	/**
	 * Original PX4 position controller retained
	 * but unused in the full-SMC path.
	 */

	Vector3f vel_sp_position =

		(_pos_sp - _pos)

		.emult(
			_gain_pos_p);


	ControlMath::addIfNotNanVector3f(

		_vel_sp,

		vel_sp_position);


	ControlMath::setZeroIfNanVector3f(

		vel_sp_position);


	_vel_sp.xy() =

		ControlMath::constrainXY(

			vel_sp_position.xy(),

			(_vel_sp
			 - vel_sp_position).xy(),

			_lim_vel_horizontal);


	_vel_sp(2) =

		math::constrain(

			_vel_sp(2),

			-_lim_vel_up,

			_lim_vel_down);
}


void
PositionControl::_velocityControl(
	const float dt)
{
	/**
	 * Original PX4 velocity controller retained
	 * but unused by the full-SMC path.
	 */

	_vel_int(2) =

		math::constrain(

			_vel_int(2),

			-CONSTANTS_ONE_G,

			CONSTANTS_ONE_G);


	Vector3f vel_error =

		_vel_sp

		- _vel;


	Vector3f acc_sp_velocity =

		vel_error.emult(
			_gain_vel_p)

		+ _vel_int

		- _vel_dot.emult(
			_gain_vel_d);


	ControlMath::addIfNotNanVector3f(

		_acc_sp,

		acc_sp_velocity);


	_accelerationControl();


	if ((_thr_sp(2)
	     >= -_lim_thr_min

	     && vel_error(2)
	     >= 0.f)

	    ||

	    (_thr_sp(2)
	     <= -_lim_thr_max

	     && vel_error(2)
	     <= 0.f)) {

		vel_error(2) =
			0.f;
	}


	const Vector2f thrust_sp_xy(
		_thr_sp);


	const float thrust_sp_xy_norm =
		thrust_sp_xy.norm();


	const float thrust_max_squared =
		math::sq(
			_lim_thr_max);


	const float allocated_horizontal_thrust =

		math::min(

			thrust_sp_xy_norm,

			_lim_thr_xy_margin);


	const float thrust_z_max_squared =

		thrust_max_squared

		- math::sq(
			allocated_horizontal_thrust);


	_thr_sp(2) =

		math::max(

			_thr_sp(2),

			-sqrtf(
				thrust_z_max_squared));


	const float thrust_max_xy_squared =

		thrust_max_squared

		- math::sq(
			_thr_sp(2));


	float thrust_max_xy =
		0.f;


	if (thrust_max_xy_squared
	    > 0.f) {

		thrust_max_xy =

			sqrtf(
				thrust_max_xy_squared);
	}


	if (thrust_sp_xy_norm
	    > thrust_max_xy) {

		_thr_sp.xy() =

			thrust_sp_xy

			/ thrust_sp_xy_norm

			* thrust_max_xy;
	}


	const Vector2f acc_sp_xy_produced =

		Vector2f(
			_thr_sp)

		* (CONSTANTS_ONE_G
		   / _hover_thrust);


	if (_acc_sp.xy().norm_squared()

	    >

	    acc_sp_xy_produced.norm_squared()) {

		const float arw_gain =

			2.f

			/ _gain_vel_p(0);


		const Vector2f acc_sp_xy =

			_acc_sp.xy();


		vel_error.xy() =

			Vector2f(
				vel_error)

			- arw_gain

			* (acc_sp_xy
			   - acc_sp_xy_produced);
	}


	ControlMath::setZeroIfNanVector3f(
		vel_error);


	_vel_int +=

		vel_error.emult(
			_gain_vel_i)

		* dt;
}


void
PositionControl::_accelerationControl()
{
	// =========================================================
	// Standard PX4 acceleration -> thrust mapping
	// =========================================================

	float z_specific_force =
		-CONSTANTS_ONE_G;


	if (!_decouple_horizontal_and_vertical_acceleration) {

		z_specific_force +=
			_acc_sp(2);
	}


	Vector3f body_z =

		Vector3f(

			-_acc_sp(0),

			-_acc_sp(1),

			-z_specific_force)

		.normalized();


	ControlMath::limitTilt(

		body_z,

		Vector3f(
			0.f,
			0.f,
			1.f),

		_lim_tilt);


	const float thrust_ned_z =

		_acc_sp(2)

		* (_hover_thrust
		   / CONSTANTS_ONE_G)

		- _hover_thrust;


	const float cos_ned_body =

		Vector3f(
			0.f,
			0.f,
			1.f)

		.dot(
			body_z);


	const float collective_thrust =

		math::min(

			thrust_ned_z

			/ cos_ned_body,

			-_lim_thr_min);


	_thr_sp =

		body_z

		* collective_thrust;
}


bool
PositionControl::_inputValid()
{
	bool valid =
		true;


	for (int i = 0;
	     i <= 2;
	     i++) {

		valid =

			valid

			&&
			(
				PX4_ISFINITE(
					_pos_sp(i))

				||

				PX4_ISFINITE(
					_vel_sp(i))

				||

				PX4_ISFINITE(
					_acc_sp(i))
			);
	}


	valid =

		valid

		&&
		(
			PX4_ISFINITE(
				_pos_sp(0))

			==

			PX4_ISFINITE(
				_pos_sp(1))
		);


	valid =

		valid

		&&
		(
			PX4_ISFINITE(
				_vel_sp(0))

			==

			PX4_ISFINITE(
				_vel_sp(1))
		);


	valid =

		valid

		&&
		(
			PX4_ISFINITE(
				_acc_sp(0))

			==

			PX4_ISFINITE(
				_acc_sp(1))
		);


	for (int i = 0;
	     i <= 2;
	     i++) {

		if (PX4_ISFINITE(
			    _pos_sp(i))) {

			valid =

				valid

				&& PX4_ISFINITE(
					_pos(i));
		}


		if (PX4_ISFINITE(
			    _vel_sp(i))) {

			valid =

				valid

				&& PX4_ISFINITE(
					_vel(i))

				&& PX4_ISFINITE(
					_vel_dot(i));
		}
	}


	return valid;
}


void
PositionControl::getLocalPositionSetpoint(
	vehicle_local_position_setpoint_s
	&local_position_setpoint) const
{
	local_position_setpoint.x =
		_pos_sp(0);


	local_position_setpoint.y =
		_pos_sp(1);


	local_position_setpoint.z =
		_pos_sp(2);


	local_position_setpoint.yaw =
		_yaw_sp;


	local_position_setpoint.yawspeed =
		_yawspeed_sp;


	local_position_setpoint.vx =
		_vel_sp(0);


	local_position_setpoint.vy =
		_vel_sp(1);


	local_position_setpoint.vz =
		_vel_sp(2);


	_acc_sp.copyTo(
		local_position_setpoint.acceleration);


	_thr_sp.copyTo(
		local_position_setpoint.thrust);
}


void
PositionControl::getAttitudeSetpoint(
	vehicle_attitude_setpoint_s
	&attitude_setpoint)
{
	// =========================================================
	// Desired attitude q_d
	// =========================================================

	if (_attitude_reference_valid
	    && !_attitude_reference_saturated) {

		/**
		 * Use q_d from the exact same R_d
		 * that generated omega_Rd and alpha_Rd.
		 */
		const Quatf q_sp{
			_R_d
		};


		q_sp.copyTo(
			attitude_setpoint.q_d);


		/**
		 * PX4 attitude setpoint convention:
		 * thrust is expressed along body z.
		 */
		attitude_setpoint.thrust_body[0] =
			0.f;


		attitude_setpoint.thrust_body[1] =
			0.f;


		attitude_setpoint.thrust_body[2] =
			-_thr_sp.length();


	} else {

		/**
		 * Safety fallback.
		 *
		 * Preserve original PX4 thrust-to-attitude
		 * mapping whenever the analytical derivatives
		 * have been invalidated by a limiter.
		 */
		ControlMath::thrustToAttitude(

			_thr_sp,

			_yaw_sp,

			attitude_setpoint);


		/**
		 * Keep internal R_d consistent with q_d
		 * during fallback.
		 */
		_R_d =

			Dcmf(

				Quatf(
					attitude_setpoint.q_d));


		_omega_Rd.zero();

		_omega_dot_Rd.zero();
	}


	attitude_setpoint.yaw_sp_move_rate =
		_yawspeed_sp;


	// =========================================================
	// Publish complete analytical reference atomically
	// =========================================================

	smc_attitude_reference_s ref{};


	ref.timestamp =
		hrt_absolute_time();


	for (int i = 0;
	     i < 4;
	     i++) {

		ref.q_d[i] =
			attitude_setpoint.q_d[i];
	}


	for (int i = 0;
	     i < 3;
	     i++) {

		ref.omega_rd[i] =
			_omega_Rd(i);


		ref.alpha_rd[i] =
			_omega_dot_Rd(i);


		ref.thrust_body[i] =
			attitude_setpoint.thrust_body[i];
	}


	const Quatf q_check{
		attitude_setpoint.q_d
	};


	ref.valid =

		q_check.isAllFinite()

		&& Vector3f(
			attitude_setpoint.thrust_body)
		.isAllFinite();


	ref.analytic_valid =
		_attitude_reference_valid;


	ref.saturated =
		_attitude_reference_saturated;


	_smc_attitude_reference_pub.publish(
		ref);
}