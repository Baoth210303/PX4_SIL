/****************************************************************************
 *
 *   Multicopter quaternion SMC controller.
 *
 ****************************************************************************/

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/smc_attitude_reference.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>


using namespace time_literals;


class MulticopterAttSmcControl :
	public ModuleBase<MulticopterAttSmcControl>,
	public ModuleParams,
	public px4::WorkItem
{
public:

	MulticopterAttSmcControl(
		bool vtol = false);


	~MulticopterAttSmcControl() override;


	static int task_spawn(
		int argc,
		char *argv[]);


	static int custom_command(
		int argc,
		char *argv[]);


	static int print_usage(
		const char *reason = nullptr);


	bool init();


private:

	void Run() override;


	void parameters_updated();


	void updateActuatorControlsStatus(
		const vehicle_torque_setpoint_s
		&vehicle_torque_setpoint,
		float dt);


	matrix::Vector3f saturation(
		const matrix::Vector3f &s) const;


	// =========================================================
	// uORB subscriptions
	// =========================================================

	uORB::Subscription
	_vehicle_attitude_sub{
		ORB_ID(vehicle_attitude)
	};


	uORB::Subscription
	_smc_attitude_reference_sub{
		ORB_ID(smc_attitude_reference)
	};


	uORB::Subscription
	_vehicle_control_mode_sub{
		ORB_ID(vehicle_control_mode)
	};


	uORB::Subscription
	_vehicle_land_detected_sub{
		ORB_ID(vehicle_land_detected)
	};


	uORB::Subscription
	_vehicle_status_sub{
		ORB_ID(vehicle_status)
	};


	uORB::SubscriptionInterval
	_parameter_update_sub{
		ORB_ID(parameter_update),
		1_s
	};


	uORB::SubscriptionCallbackWorkItem
	_vehicle_angular_velocity_sub{
		this,
		ORB_ID(vehicle_angular_velocity)
	};


	// =========================================================
	// uORB publications
	// =========================================================

	uORB::Publication<actuator_controls_status_s>
	_actuator_controls_status_pub{
		ORB_ID(actuator_controls_status_0)
	};


	uORB::Publication<vehicle_thrust_setpoint_s>
	_vehicle_thrust_setpoint_pub;


	uORB::Publication<vehicle_torque_setpoint_s>
	_vehicle_torque_setpoint_pub;


	// =========================================================
	// PX4 states
	// =========================================================

	vehicle_control_mode_s
	_vehicle_control_mode{};


	vehicle_status_s
	_vehicle_status{};


	bool _landed{
		true
	};


	bool _maybe_landed{
		true
	};


	bool _attitude_valid{
		false
	};


	bool _attitude_ref_valid{
		false
	};


	bool _reference_analytic_valid{
		false
	};


	bool _reference_saturated{
		false
	};


	bool _error_derivative_initialized{
		false
	};


	// =========================================================
	// Timing
	// =========================================================

	hrt_abstime _last_run{
		0
	};


	hrt_abstime _last_reference_update{
		0
	};


	static constexpr hrt_abstime
	REFERENCE_TIMEOUT_US =
		100_ms;


	perf_counter_t
	_loop_perf;


	// =========================================================
	// Actual and desired attitudes
	// =========================================================

	matrix::Quatf _q{
		1.f,
		0.f,
		0.f,
		0.f
	};


	matrix::Quatf _q_d{
		1.f,
		0.f,
		0.f,
		0.f
	};


	// =========================================================
	// Desired position-loop attitude trajectory
	//
	// omega_Rd:
	// desired angular velocity in desired body B_d.
	//
	// alpha_Rd:
	// desired angular acceleration in desired body B_d.
	// =========================================================

	matrix::Vector3f
	_omega_Rd{};


	matrix::Vector3f
	_alpha_Rd{};


	matrix::Vector3f
	_thrust_setpoint{};


	// =========================================================
	// Quaternion-error derivative
	//
	// This filter is intentionally retained.
	//
	// It is NOT an LPF on desired omega_d or alpha_d.
	// =========================================================

	matrix::Vector3f
	_e_q_prev{};


	matrix::Vector3f
	_e_q_dot_filt{};


	float _e_dot_lpf_cutoff{
		5.f
	};


	// =========================================================
	// Attitude SMC gains
	//
	// Keep current verified values.
	// =========================================================

	matrix::Vector3f _smc_lambda{
		3.5f,
		3.5f,
		2.5f
	};


	matrix::Vector3f _smc_k{
		0.04f,
		0.028f,
		0.03f
	};


	matrix::Vector3f _smc_eta{
		0.0025f,
		0.0025f,
		0.0014f
	};


	matrix::Vector3f _smc_boundary{
		0.40f,
		0.40f,
		0.30f
	};


	// =========================================================
	// X500 composite inertia [kg.m^2]
	// =========================================================

	matrix::Vector3f _inertia{
		0.02383948f,
		0.02394241f,
		0.04399995f
	};


	// =========================================================
	// Physical torque normalization limits [Nm]
	// =========================================================

	matrix::Vector3f _torque_max{
		0.40f,
		0.40f,
		0.20f
	};


	// =========================================================
	// Control effort monitoring
	// =========================================================

	float _energy_integration_time{
		0.0f
	};


	float _control_energy[4]{};
};