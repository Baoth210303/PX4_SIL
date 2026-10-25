/****************************************************************************
 *
 *   Quaternion Sliding Mode Attitude Controller (QSMC)
 *   for PX4 multicopter attitude control.
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

	// =========================================================
	// Main controller loop
	// =========================================================

	void Run() override;

	void parameters_updated();

	void updateActuatorControlsStatus(
		const vehicle_torque_setpoint_s &vehicle_torque_setpoint,
		float dt);


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
	// PX4 vehicle states
	// =========================================================

	vehicle_control_mode_s _vehicle_control_mode{};

	vehicle_status_s _vehicle_status{};

	bool _landed{true};

	bool _maybe_landed{true};

	bool _attitude_valid{false};

	bool _attitude_sp_valid{false};


	// =========================================================
	// Timing
	// =========================================================

	hrt_abstime _last_run{0};

	perf_counter_t _loop_perf;


	// =========================================================
	// Actual and desired attitude quaternions
	//
	// q   : actual attitude quaternion
	// q_d : desired attitude quaternion
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
	// Desired attitude kinematics
	//
	// The PX4 attitude setpoint provides q_d.
	//
	// omega_Rd:
	// desired angular velocity expressed in the desired frame.
	//
	// alpha_Rd:
	// desired angular acceleration expressed in the desired frame.
	//
	// These quantities are estimated from successive q_d samples.
	// =========================================================

	matrix::Quatf _q_d_prev{
		1.f,
		0.f,
		0.f,
		0.f
	};

	hrt_abstime _q_d_prev_timestamp{0};

	matrix::Vector3f _omega_Rd{};

	matrix::Vector3f _alpha_Rd_filt{};

	bool _desired_kinematics_initialized{false};


	// =========================================================
	// Thrust setpoint
	//
	// QSMC controls attitude torque only.
	// The thrust command is passed through from
	// vehicle_attitude_setpoint.
	// =========================================================

	matrix::Vector3f _thrust_setpoint{};


	// =========================================================
	// QSMC gains
	//
	// Sliding surface:
	//
	// s_q =
	// omega_e
	// + Lambda_q * sgn_+(q_we) * q_e_vec
	//
	// Control law:
	//
	// tau =
	// J_hat * omega_dot_d
	// + omega x (J_hat * omega)
	// - J_hat * Lambda_q * sgn_+(q_we) * q_e_vec_dot
	// - J_hat * K_q * tanh(s_q / phi_q)
	//
	// =========================================================


	// Lambda_q
	//
	// Sliding surface convergence gain
	//
	matrix::Vector3f _smc_lambda{
		2.0f,
		2.4f,
		1.3f
	};


	// K_q
	//
	// QSMC switching / robust gain
	//
	// NOTE:
	// These are initial commissioning values only.
	// They will be retuned for the X500.
	//
	matrix::Vector3f _smc_k{
		0.016f,
		0.028f,
		0.007f
	};


	// phi_q
	//
	// Boundary-layer parameters used in:
	//
	// tanh(s_q / phi_q)
	//
	// The variable name "_smc_boundary" is temporarily retained
	// to minimize unnecessary code changes.
	//
	matrix::Vector3f _smc_boundary{
		0.40f,
		0.40f,
		0.30f
	};


	// =========================================================
	// Nominal inertia matrix
	//
	// J_hat = diag(Jx, Jy, Jz)
	//
	// Approximate composite inertia of Gazebo X500 [kg.m^2]
	// =========================================================

	matrix::Vector3f _inertia{
		0.02385f,
		0.02395f,
		0.04400f
	};


	// =========================================================
	// Torque normalization
	//
	// Physical torque [Nm] is converted to the normalized
	// PX4 vehicle_torque_setpoint range [-1, 1].
	// =========================================================

	matrix::Vector3f _torque_max{
		0.40f,
		0.40f,
		0.20f
	};


	// =========================================================
	// Desired angular acceleration LPF
	// =========================================================

	float _omega_d_dot_lpf_cutoff{5.f};


	// =========================================================
	// Control effort monitoring
	// =========================================================

	float _energy_integration_time{0.0f};

	float _control_energy[4]{};
};