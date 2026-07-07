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

class MulticopterAttSmcControl : public ModuleBase<MulticopterAttSmcControl>, public ModuleParams, public px4::WorkItem
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

	void updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint, float dt);

	matrix::Vector3f saturation(const matrix::Vector3f &s) const;

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::Publication<actuator_controls_status_s> _actuator_controls_status_pub{ORB_ID(actuator_controls_status_0)};
	uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub;
	uORB::Publication<vehicle_torque_setpoint_s> _vehicle_torque_setpoint_pub;

	vehicle_control_mode_s _vehicle_control_mode{};
	vehicle_status_s _vehicle_status{};

	bool _landed{true};
	bool _maybe_landed{true};

	bool _attitude_valid{false};
	bool _attitude_sp_valid{false};
	bool _error_derivative_initialized{false};

	hrt_abstime _last_run{0};

	perf_counter_t _loop_perf;

	matrix::Quatf _q{1.f, 0.f, 0.f, 0.f};
	matrix::Quatf _q_d{1.f, 0.f, 0.f, 0.f};

	matrix::Vector3f _thrust_setpoint{};
	float _yaw_sp_move_rate{0.f};

	matrix::Vector3f _e_q_prev{};
	matrix::Vector3f _e_q_dot_filt{};

	// Initial SMC gains for SITL verification.
	matrix::Vector3f _smc_lambda{1.5f, 1.5f, 1.0f};
	matrix::Vector3f _smc_k{0.008f, 0.008f, 0.004f};
	matrix::Vector3f _smc_eta{0.0015f, 0.0015f, 0.0010f};
	matrix::Vector3f _smc_boundary{0.40f, 0.40f, 0.30f};

	// Approximate inertia from the MATLAB/SITL model [kg.m^2].
	matrix::Vector3f _inertia{4.856e-3f, 4.856e-3f, 8.801e-3f};

	// Torque normalization values [Nm].
	matrix::Vector3f _torque_max{0.40f, 0.40f, 0.20f};

	float _e_dot_lpf_cutoff{5.f};

	float _energy_integration_time{0.0f};
	float _control_energy[4]{};
};
