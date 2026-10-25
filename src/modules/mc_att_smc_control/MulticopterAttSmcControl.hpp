/****************************************************************************
 *
 *   Multicopter attitude SMC controller for PX4 SITL verification.
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
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
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

	MulticopterAttSmcControl(bool vtol = false);

	~MulticopterAttSmcControl() override;

	static int task_spawn(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	static int print_usage(const char *reason = nullptr);

	bool init();


private:

	void Run() override;

	void parameters_updated();

	void updateActuatorControlsStatus(
		const vehicle_torque_setpoint_s &vehicle_torque_setpoint,
		float dt);

	matrix::Vector3f saturation(
		const matrix::Vector3f &s) const;


	// =========================================================
	// uORB subscriptions
	// =========================================================

	uORB::Subscription _vehicle_attitude_sub{
		ORB_ID(vehicle_attitude)
	};

	uORB::Subscription _vehicle_attitude_setpoint_sub{
		ORB_ID(vehicle_attitude_setpoint)
	};

	uORB::Subscription _vehicle_control_mode_sub{
		ORB_ID(vehicle_control_mode)
	};

	uORB::Subscription _vehicle_land_detected_sub{
		ORB_ID(vehicle_land_detected)
	};

	uORB::Subscription _vehicle_status_sub{
		ORB_ID(vehicle_status)
	};

	uORB::SubscriptionInterval _parameter_update_sub{
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

	vehicle_control_mode_s _vehicle_control_mode{};

	vehicle_status_s _vehicle_status{};

	bool _landed{true};

	bool _maybe_landed{true};

	bool _attitude_valid{false};

	bool _attitude_sp_valid{false};

	bool _error_derivative_initialized{false};


	// =========================================================
	// Timing
	// =========================================================

	hrt_abstime _last_run{0};

	perf_counter_t _loop_perf;


	// =========================================================
	// Quaternion states
	//
	// q:
	// actual attitude from vehicle_attitude / EKF
	//
	// q_d:
	// desired attitude from vehicle_attitude_setpoint
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
	// PX4 attitude setpoint
	// =========================================================

	matrix::Vector3f _thrust_setpoint{};

	float _yaw_sp_move_rate{0.f};


	// =========================================================
	// Quaternion error derivative
	//
	// Keep old numerical derivative + LPF structure.
	// =========================================================

	matrix::Vector3f _e_q_prev{};

	matrix::Vector3f _e_q_dot_filt{};


	// =========================================================
	// Desired angular velocity
	//
	// Keep old PX4 implementation.
	// =========================================================

	matrix::Vector3f _omega_d_prev{};

	matrix::Vector3f _omega_d_dot_filt{};

	bool _omega_d_derivative_initialized{false};


	// =========================================================
	// SMC gains
	//
	// Keep exactly the old stable gains.
	// =========================================================

	matrix::Vector3f _smc_lambda{
		2.0f,
		2.4f,
		1.3f
	};

	matrix::Vector3f _smc_k{
		0.016f,
		0.028f,
		0.007f
	};

	matrix::Vector3f _smc_eta{
		0.0025f,
		0.0040f,
		0.0014f
	};

	matrix::Vector3f _smc_boundary{
		0.40f,
		0.40f,
		0.30f
	};


	// =========================================================
	// X500 composite inertia
	//
	// Calculated from x500_base.sdf:
	//
	// base_link + 4 rotors
	//
	// expressed about the complete vehicle CoM.
	//
	// [kg.m^2]
	// =========================================================

	matrix::Vector3f _inertia{
		0.02383948f,
		0.02394241f,
		0.04399995f
	};


	// =========================================================
	// Torque normalization
	//
	// Keep old values.
	//
	// [Nm]
	// =========================================================

	matrix::Vector3f _torque_max{
		0.40f,
		0.40f,
		0.20f
	};


	// =========================================================
	// Filters
	// =========================================================

	float _e_dot_lpf_cutoff{5.f};

	float _omega_d_dot_lpf_cutoff{5.f};


	// =========================================================
	// Control effort monitoring
	// =========================================================

	float _energy_integration_time{0.0f};

	float _control_energy[4]{};
};