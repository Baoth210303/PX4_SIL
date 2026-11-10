/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#pragma once

#include <lib/rate_control/rate_control.hpp>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/systemlib/mavlink_log.h>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>

#include <uORB/topics/smc_attitude_reference.h>

#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;

class MulticopterRateControl :
	public ModuleBase<MulticopterRateControl>,
	public ModuleParams,
	public px4::WorkItem
{
public:
	MulticopterRateControl(bool vtol = false);
	~MulticopterRateControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	/**
	 * Initialize vectors/matrices from parameters.
	 *
	 * Stock PX4 rate-control parameters remain available for
	 * ACRO/manual operation and fallback.
	 */
	void parameters_updated();

	void updateActuatorControlsStatus(
		const vehicle_torque_setpoint_s &vehicle_torque_setpoint,
		float dt);

	/**
	 * Component-wise boundary-layer saturation:
	 *
	 * sat_i = constrain(s_i / phi_i, -1, 1)
	 */
	matrix::Vector3f smcSaturation(
		const matrix::Vector3f &s) const;

	/*
	 * =========================================================
	 * Stock PX4 rate controller
	 * =========================================================
	 *
	 * Retained for:
	 *
	 * - ACRO/manual rate control
	 * - fallback when SMC reference is invalid
	 * - fallback when reference is stale
	 * - fallback when analytical trajectory is saturated
	 */
	RateControl _rate_control;

	/*
	 * =========================================================
	 * Stock PX4 subscriptions
	 * =========================================================
	 */

	uORB::Subscription _battery_status_sub{
		ORB_ID(battery_status)
	};

	uORB::Subscription _control_allocator_status_sub{
		ORB_ID(control_allocator_status)
	};

	uORB::Subscription _manual_control_setpoint_sub{
		ORB_ID(manual_control_setpoint)
	};

	uORB::Subscription _vehicle_control_mode_sub{
		ORB_ID(vehicle_control_mode)
	};

	uORB::Subscription _vehicle_land_detected_sub{
		ORB_ID(vehicle_land_detected)
	};

	uORB::Subscription _vehicle_rates_setpoint_sub{
		ORB_ID(vehicle_rates_setpoint)
	};

	uORB::Subscription _vehicle_status_sub{
		ORB_ID(vehicle_status)
	};

	/*
	 * =========================================================
	 * Quaternion SMC subscriptions
	 * =========================================================
	 *
	 * Current attitude:
	 *
	 * vehicle_attitude
	 *
	 * Desired analytical attitude trajectory:
	 *
	 * smc_attitude_reference
	 *
	 *     q_d
	 *     omega_Rd
	 *     alpha_Rd
	 */
	uORB::Subscription _vehicle_attitude_sub{
		ORB_ID(vehicle_attitude)
	};

	uORB::Subscription _smc_attitude_reference_sub{
		ORB_ID(smc_attitude_reference)
	};

	uORB::SubscriptionInterval _parameter_update_sub{
		ORB_ID(parameter_update),
		1_s
	};

	/*
	 * Main rate-control callback remains exactly the stock
	 * vehicle_angular_velocity callback on wq:rate_ctrl.
	 */
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{
		this,
		ORB_ID(vehicle_angular_velocity)
	};

	/*
	 * =========================================================
	 * Stock PX4 publications
	 * =========================================================
	 */

	uORB::Publication<actuator_controls_status_s>
	_actuator_controls_status_pub{
		ORB_ID(actuator_controls_status_0)
	};

	uORB::PublicationMulti<rate_ctrl_status_s>
	_controller_status_pub{
		ORB_ID(rate_ctrl_status)
	};

	uORB::Publication<vehicle_rates_setpoint_s>
	_vehicle_rates_setpoint_pub{
		ORB_ID(vehicle_rates_setpoint)
	};

	uORB::Publication<vehicle_thrust_setpoint_s>
	_vehicle_thrust_setpoint_pub;

	uORB::Publication<vehicle_torque_setpoint_s>
	_vehicle_torque_setpoint_pub;

	/*
	 * =========================================================
	 * ESO diagnostic publication
	 * =========================================================
	 *
	 * debug_array.data:
	 *
	 * data[0] = omega_hat_x
	 * data[1] = omega_hat_y
	 * data[2] = omega_hat_z
	 *
	 * data[3] = d_omega_hat_x
	 * data[4] = d_omega_hat_y
	 * data[5] = d_omega_hat_z
	 *
	 * data[6] = tau_ESO_x
	 * data[7] = tau_ESO_y
	 * data[8] = tau_ESO_z
	 *
	 * where:
	 *
	 * tau_ESO = -J * d_omega_hat
	 */
	uORB::Publication<debug_array_s> _eso_debug_pub{
		ORB_ID(debug_array)
	};

	/*
	 * =========================================================
	 * PX4 state
	 * =========================================================
	 */

	vehicle_control_mode_s _vehicle_control_mode{};
	vehicle_status_s _vehicle_status{};

	bool _landed{true};
	bool _maybe_landed{true};

	/*
	 * =========================================================
	 * SMC validity state
	 * =========================================================
	 */

	bool _smc_attitude_valid{false};

	bool _smc_reference_valid{false};

	bool _smc_reference_saturated{false};

	/*
	 * =========================================================
	 * Timing
	 * =========================================================
	 */

	hrt_abstime _last_run{0};

	hrt_abstime _last_smc_reference_update{0};

	hrt_abstime _last_attitude_update{0};

	/*
	 * Last ESO diagnostic publication timestamp.
	 */
	hrt_abstime _last_eso_debug_publish{0};

	/*
	 * ESO diagnostic publication interval:
	 *
	 * 10 ms = 100 Hz
	 */
	static constexpr hrt_abstime
	ESO_DEBUG_INTERVAL = 10_ms;

	static constexpr hrt_abstime
	SMC_REFERENCE_TIMEOUT = 100_ms;

	static constexpr hrt_abstime
	ATTITUDE_TIMEOUT = 100_ms;

	perf_counter_t _loop_perf;

	/*
	 * =========================================================
	 * Stock PX4 rate/thrust setpoints
	 * =========================================================
	 */

	matrix::Vector3f _acro_rate_max;

	matrix::Vector3f _rates_setpoint{};

	float _battery_status_scale{0.0f};

	matrix::Vector3f _thrust_setpoint{};

	/*
	 * =========================================================
	 * Current attitude
	 * =========================================================
	 *
	 * q:
	 *
	 * current body attitude from EKF / vehicle_attitude
	 */
	matrix::Quatf _q{
		1.f,
		0.f,
		0.f,
		0.f
	};

	/*
	 * =========================================================
	 * Desired attitude
	 * =========================================================
	 *
	 * q_d:
	 *
	 * desired quaternion generated analytically by
	 * mc_att_control.
	 */
	matrix::Quatf _q_d{
		1.f,
		0.f,
		0.f,
		0.f
	};

	/*
	 * =========================================================
	 * Desired angular trajectory
	 * =========================================================
	 *
	 * omega_Rd:
	 *
	 * desired angular velocity expressed in desired
	 * body frame B_d.
	 *
	 *
	 * alpha_Rd:
	 *
	 * desired angular acceleration expressed in desired
	 * body frame B_d.
	 */
	matrix::Vector3f _omega_Rd{};

	matrix::Vector3f _alpha_Rd{};

	/*
	 * =========================================================
	 * Quaternion SMC gains
	 * =========================================================
	 *
	 * Conservative F450 bring-up values.
	 *
	 * Original X500 values:
	 *
	 * lambda = [3.5, 3.5, 2.5]
	 *
	 * K      = [0.040, 0.028, 0.030]
	 *
	 * eta    = [0.0025, 0.0025, 0.0014]
	 *
	 * phi    = [0.40, 0.40, 0.30]
	 *
	 *
	 * F450 bring-up:
	 *
	 * lower Lambda
	 * lower reaching gains
	 * wider boundary layer
	 */
	matrix::Vector3f _smc_lambda{
		2.4f,
		2.4f,
		1.5f
	};

	matrix::Vector3f _smc_k{
		0.018f,
		0.018f,
		0.010f
	};

	matrix::Vector3f _smc_eta{
		0.025f,
		0.025f,
		0.018f
	};

	matrix::Vector3f _smc_boundary{
		0.50f,
		0.50f,
		0.40f
	};

	/*
	 * =========================================================
	 * F450 composite inertia [kg m^2]
	 * =========================================================
	 *
	 * Temporary initial approximation:
	 *
	 * J_F450 ≈ 0.5 J_X500
	 *
	 *
	 * Previous X500:
	 *
	 * Jx = 0.02383948
	 *
	 * Jy = 0.02394241
	 *
	 * Jz = 0.04399995
	 *
	 *
	 * Initial F450 estimate:
	 *
	 * Jx = 0.01191974
	 *
	 * Jy = 0.01197121
	 *
	 * Jz = 0.02199998
	 *
	 * These should later be replaced by measured or
	 * identified F450 inertia.
	 */
	matrix::Vector3f _smc_inertia{
		0.01191974f,
		0.01197121f,
		0.02199998f
	};

	/*
	 * =========================================================
	 * Physical torque normalization limits [Nm]
	 * =========================================================
	 *
	 * Keep the previous normalization values for the
	 * initial structural port.
	 *
	 * Do NOT simply divide these values by two when
	 * changing from X500 to F450 because:
	 *
	 * normalized torque = physical torque / torque_max
	 *
	 * Reducing torque_max would increase the normalized
	 * PX4 command for the same requested physical torque.
	 */
	matrix::Vector3f _smc_torque_max{
		0.40f,
		0.40f,
		0.20f
	};

	/*
	 * =========================================================
	 * Attitude model-based ESO
	 * =========================================================
	 *
	 * Plant:
	 *
	 * omega_dot =
	 *     J^{-1}(tau - omega x J omega)
	 *     + d_omega
	 *
	 * Observer:
	 *
	 * omega_hat_dot =
	 *     f_omega
	 *     + d_omega_hat
	 *     + L1 .* (omega - omega_hat)
	 *
	 * d_omega_hat_dot =
	 *     L2 .* (omega - omega_hat)
	 *
	 * Per-axis observer bandwidth:
	 *
	 * w_o = [w_ox, w_oy, w_oz]^T
	 *
	 * L1 = 2*w_o
	 *
	 * L2 = w_o .* w_o
	 *
	 * ESO is updated directly in the existing
	 * vehicle_angular_velocity callback using the actual PX4 dt.
	 *
	 * Previous-iteration physical torque is used by the observer
	 * to avoid an algebraic loop with the current QSMC torque law.
	 */
	matrix::Vector3f _eso_attitude_bandwidth{
		15.0f,
		15.0f,
		15.0f
	};

	matrix::Vector3f _eso_compensation_gain{
    	0.25f,
    	0.30f,
    	0.25f
	};

	matrix::Vector3f _eso_omega_hat{};

	matrix::Vector3f _eso_domega_hat{};

	matrix::Vector3f _eso_tau_prev{};

	bool _eso_attitude_initialized{false};

	/*
	 * =========================================================
	 * Control effort monitoring
	 * =========================================================
	 */

	float _energy_integration_time{0.0f};

	float _control_energy[4] {};

	/*
	 * =========================================================
	 * Stock yaw torque filter
	 * =========================================================
	 *
	 * Retained for the stock PID fallback path.
	 *
	 * The analytical QSMC path will not filter:
	 *
	 * omega_d
	 * alpha_d
	 *
	 * and will not require numerical differentiation of
	 * the desired angular reference.
	 */
	AlphaFilter<float> _output_lpf_yaw;

	/*
	 * =========================================================
	 * Original PX4 parameters
	 * =========================================================
	 */

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_ROLLRATE_P>)
		_param_mc_rollrate_p,

		(ParamFloat<px4::params::MC_ROLLRATE_I>)
		_param_mc_rollrate_i,

		(ParamFloat<px4::params::MC_RR_INT_LIM>)
		_param_mc_rr_int_lim,

		(ParamFloat<px4::params::MC_ROLLRATE_D>)
		_param_mc_rollrate_d,

		(ParamFloat<px4::params::MC_ROLLRATE_FF>)
		_param_mc_rollrate_ff,

		(ParamFloat<px4::params::MC_ROLLRATE_K>)
		_param_mc_rollrate_k,

		(ParamFloat<px4::params::MC_PITCHRATE_P>)
		_param_mc_pitchrate_p,

		(ParamFloat<px4::params::MC_PITCHRATE_I>)
		_param_mc_pitchrate_i,

		(ParamFloat<px4::params::MC_PR_INT_LIM>)
		_param_mc_pr_int_lim,

		(ParamFloat<px4::params::MC_PITCHRATE_D>)
		_param_mc_pitchrate_d,

		(ParamFloat<px4::params::MC_PITCHRATE_FF>)
		_param_mc_pitchrate_ff,

		(ParamFloat<px4::params::MC_PITCHRATE_K>)
		_param_mc_pitchrate_k,

		(ParamFloat<px4::params::MC_YAWRATE_P>)
		_param_mc_yawrate_p,

		(ParamFloat<px4::params::MC_YAWRATE_I>)
		_param_mc_yawrate_i,

		(ParamFloat<px4::params::MC_YR_INT_LIM>)
		_param_mc_yr_int_lim,

		(ParamFloat<px4::params::MC_YAWRATE_D>)
		_param_mc_yawrate_d,

		(ParamFloat<px4::params::MC_YAWRATE_FF>)
		_param_mc_yawrate_ff,

		(ParamFloat<px4::params::MC_YAWRATE_K>)
		_param_mc_yawrate_k,

		(ParamFloat<px4::params::MC_YAW_TQ_CUTOFF>)
		_param_mc_yaw_tq_cutoff,

		(ParamFloat<px4::params::MC_ACRO_R_MAX>)
		_param_mc_acro_r_max,

		(ParamFloat<px4::params::MC_ACRO_P_MAX>)
		_param_mc_acro_p_max,

		(ParamFloat<px4::params::MC_ACRO_Y_MAX>)
		_param_mc_acro_y_max,

		(ParamFloat<px4::params::MC_ACRO_EXPO>)
		_param_mc_acro_expo,

		(ParamFloat<px4::params::MC_ACRO_EXPO_Y>)
		_param_mc_acro_expo_y,

		(ParamFloat<px4::params::MC_ACRO_SUPEXPO>)
		_param_mc_acro_supexpo,

		(ParamFloat<px4::params::MC_ACRO_SUPEXPOY>)
		_param_mc_acro_supexpoy,

		(ParamBool<px4::params::MC_BAT_SCALE_EN>)
		_param_mc_bat_scale_en
	)
};