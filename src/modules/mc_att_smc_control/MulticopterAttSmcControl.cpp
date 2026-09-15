/****************************************************************************
 *
 *   Multicopter attitude SMC controller for PX4 SITL verification.
 *
 ****************************************************************************/

#include "MulticopterAttSmcControl.hpp"

#include <float.h>
#include <math.h>
#include <string.h>

#include <mathlib/math/Functions.hpp>
#include <mathlib/math/Limits.hpp>

using namespace matrix;

MulticopterAttSmcControl::MulticopterAttSmcControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint)),
	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc) : ORB_ID(vehicle_torque_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
}

MulticopterAttSmcControl::~MulticopterAttSmcControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterAttSmcControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
MulticopterAttSmcControl::parameters_updated()
{
	// Fixed internal gains for the first SITL verification.
	// Later these can be exposed as PX4 parameters.
}

Vector3f
MulticopterAttSmcControl::saturation(const Vector3f &s) const
{
	Vector3f sat_s{};

	for (int i = 0; i < 3; i++) {
		const float boundary = fmaxf(_smc_boundary(i), FLT_EPSILON);
		sat_s(i) = math::constrain(s(i) / boundary, -1.f, 1.f);
	}

	return sat_s;
}

void
MulticopterAttSmcControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update{};
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	vehicle_angular_velocity_s angular_velocity{};

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		const hrt_abstime now = angular_velocity.timestamp_sample;
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f omega{angular_velocity.xyz};

		_vehicle_control_mode_sub.update(&_vehicle_control_mode);
		_vehicle_status_sub.update(&_vehicle_status);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected{};

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		if (_vehicle_attitude_sub.updated()) {
			vehicle_attitude_s vehicle_attitude{};

			if (_vehicle_attitude_sub.copy(&vehicle_attitude)) {
				_q = Quatf(vehicle_attitude.q);
				_attitude_valid = true;
			}
		}

		if (_vehicle_attitude_setpoint_sub.updated()) {
			vehicle_attitude_setpoint_s vehicle_attitude_setpoint{};

			if (_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint)) {
				_q_d = Quatf(vehicle_attitude_setpoint.q_d);
				_thrust_setpoint = Vector3f(vehicle_attitude_setpoint.thrust_body);
				_yaw_sp_move_rate = PX4_ISFINITE(vehicle_attitude_setpoint.yaw_sp_move_rate) ?
						    vehicle_attitude_setpoint.yaw_sp_move_rate : 0.f;

				_attitude_sp_valid = true;
			}
		}

		const bool run_smc_att_ctrl =
			_vehicle_control_mode.flag_control_attitude_enabled &&
			_vehicle_control_mode.flag_control_rates_enabled &&
			(_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);

		if (!run_smc_att_ctrl || !_attitude_valid || !_attitude_sp_valid) {
			_error_derivative_initialized = false;
			_omega_d_derivative_initialized = false;
			perf_end(_loop_perf);
			return;
		}

		// Quaternion attitude error:
		// q_error = q^{-1} * q_d
		const Quatf q_error = _q.inversed() * _q_d;
		const Vector3f e_q = 2.f * q_error.canonical().imag();

		Vector3f e_q_dot{};

		if (_error_derivative_initialized) {
			e_q_dot = (e_q - _e_q_prev) / dt;

		} else {
			e_q_dot.zero();
			_e_q_dot_filt.zero();
			_error_derivative_initialized = true;
		}

		const float alpha = expf(-6.28318530718f * _e_dot_lpf_cutoff * dt);
		_e_q_dot_filt = alpha * _e_q_dot_filt + (1.f - alpha) * e_q_dot;
		_e_q_prev = e_q;

		// Desired body rate.
		// First implementation: only yaw-rate feedforward is used.
		Vector3f omega_d{};

		if (PX4_ISFINITE(_yaw_sp_move_rate)) {
			omega_d = _q.inversed().dcm_z() * _yaw_sp_move_rate;
		}

		Vector3f omega_dot_d{};

		if (_omega_d_derivative_initialized) {
			omega_dot_d = (omega_d - _omega_d_prev) / dt;

		} else {
			omega_dot_d.zero();
			_omega_d_dot_filt.zero();
			_omega_d_derivative_initialized = true;
		}

		const float alpha_omega_d = expf(-6.28318530718f * _omega_d_dot_lpf_cutoff * dt);
		_omega_d_dot_filt = alpha_omega_d * _omega_d_dot_filt + (1.f - alpha_omega_d) * omega_dot_d;
		_omega_d_prev = omega_d;

		// Sliding surface:
		// s = omega - omega_d - Lambda * e_q
		const Vector3f s = omega - omega_d - _smc_lambda.emult(e_q);
		const Vector3f sat_s = saturation(s);

		// Newton-Euler body torque:
		// tau = I*(omega_dot_d + Lambda*e_q_dot) + omega x (I*omega) - K*s - eta*sat(s)
		const Vector3f Iomega = _inertia.emult(omega);
		const Vector3f omega_cross_Iomega = omega % Iomega;

		const Vector3f tau_phys =
			_inertia.emult(_omega_d_dot_filt + _smc_lambda.emult(_e_q_dot_filt))
			+ omega_cross_Iomega
			- _smc_k.emult(s)
			- _smc_eta.emult(sat_s);

		Vector3f torque_setpoint{};

		for (int i = 0; i < 3; i++) {
			const float tau_max = fmaxf(_torque_max(i), FLT_EPSILON);

			torque_setpoint(i) = PX4_ISFINITE(tau_phys(i)) ?
					     math::constrain(tau_phys(i) / tau_max, -1.f, 1.f) : 0.f;
		}

		vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
		vehicle_torque_setpoint_s vehicle_torque_setpoint{};

		for (int i = 0; i < 3; i++) {
			vehicle_thrust_setpoint.xyz[i] = PX4_ISFINITE(_thrust_setpoint(i)) ?
							 math::constrain(_thrust_setpoint(i), -1.f, 1.f) : 0.f;

			vehicle_torque_setpoint.xyz[i] = PX4_ISFINITE(torque_setpoint(i)) ?
							 math::constrain(torque_setpoint(i), -1.f, 1.f) : 0.f;
		}

		vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
		vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
		_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

		vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
		vehicle_torque_setpoint.timestamp = hrt_absolute_time();
		_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);

		updateActuatorControlsStatus(vehicle_torque_setpoint, dt);
	}

	perf_end(_loop_perf);
}

void
MulticopterAttSmcControl::updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint,
		float dt)
{
	for (int i = 0; i < 3; i++) {
		_control_energy[i] += vehicle_torque_setpoint.xyz[i] * vehicle_torque_setpoint.xyz[i] * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status{};
		status.timestamp = vehicle_torque_setpoint.timestamp;

		for (int i = 0; i < 3; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int
MulticopterAttSmcControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterAttSmcControl *instance = new MulticopterAttSmcControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int
MulticopterAttSmcControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
MulticopterAttSmcControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements a multicopter attitude sliding mode controller.

Inputs:
- vehicle_attitude
- vehicle_angular_velocity
- vehicle_attitude_setpoint

Outputs:
- vehicle_torque_setpoint
- vehicle_thrust_setpoint

The controller directly replaces the cascaded mc_att_control + mc_rate_control path
for SITL verification.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_att_smc_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_att_smc_control_main(int argc, char *argv[])
{
	return MulticopterAttSmcControl::main(argc, argv);
}
