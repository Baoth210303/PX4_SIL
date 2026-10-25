/****************************************************************************
 *
 *   Quaternion Sliding Mode Attitude Controller (QSMC)
 *   for PX4 multicopter attitude control.
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
	// Fixed internal gains for the first QSMC SITL verification.
	// Later these gains can be exposed as PX4 parameters.
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

	// =========================================================
	// Parameter update
	// =========================================================
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update{};
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	vehicle_angular_velocity_s angular_velocity{};

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		const hrt_abstime now = angular_velocity.timestamp_sample;

		const float dt =
			math::constrain(
				((now - _last_run) * 1e-6f),
				0.000125f,
				0.02f);

		_last_run = now;

		// Actual body angular velocity:
		//
		// omega = [p q r]^T
		//
		const Vector3f omega{angular_velocity.xyz};

		// =====================================================
		// Update PX4 states
		// =====================================================

		_vehicle_control_mode_sub.update(&_vehicle_control_mode);
		_vehicle_status_sub.update(&_vehicle_status);

		if (_vehicle_land_detected_sub.updated()) {

			vehicle_land_detected_s vehicle_land_detected{};

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {

				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		// =====================================================
		// Actual attitude quaternion q
		// =====================================================

		if (_vehicle_attitude_sub.updated()) {

			vehicle_attitude_s vehicle_attitude{};

			if (_vehicle_attitude_sub.copy(&vehicle_attitude)) {

				_q = Quatf(vehicle_attitude.q);

				_attitude_valid = true;
			}
		}

		// =====================================================
		// Desired attitude quaternion q_d
		//
		// Desired angular velocity omega_Rd and acceleration
		// alpha_Rd are estimated only when a new attitude
		// setpoint is received.
		//
		// This avoids differentiating q_d at the much faster
		// body-rate control frequency.
		// =====================================================

		if (_vehicle_attitude_setpoint_sub.updated()) {

			vehicle_attitude_setpoint_s vehicle_attitude_setpoint{};

			if (_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint)) {

				const Quatf q_d_new(vehicle_attitude_setpoint.q_d);

				const hrt_abstime setpoint_timestamp =
					vehicle_attitude_setpoint.timestamp;

				// -------------------------------------------------
				// Desired quaternion kinematics
				// -------------------------------------------------

				if (_desired_kinematics_initialized
				    && setpoint_timestamp > _q_d_prev_timestamp) {

					const float dt_sp =
						math::constrain(
							(setpoint_timestamp - _q_d_prev_timestamp)
							* 1e-6f,
							0.001f,
							0.5f);

					// Incremental desired rotation:
					//
					// delta_q_d = q_d(k-1)^* ⊗ q_d(k)
					//
					// canonical() is intentionally used here only
					// for the incremental desired quaternion so that
					// the shortest desired rotation is selected.
					//
					const Quatf delta_q_d =
						(_q_d_prev.inversed() * q_d_new).canonical();

					const Vector3f delta_q_vec =
						delta_q_d.imag();

					const float delta_q_vec_norm =
						delta_q_vec.norm();

					Vector3f omega_Rd_new{};

					if (delta_q_vec_norm > 1e-6f) {

						const float delta_angle =
							2.f * atan2f(
								delta_q_vec_norm,
								delta_q_d(0));

						omega_Rd_new =
							delta_q_vec *
							(delta_angle /
							 (delta_q_vec_norm * dt_sp));

					} else {

						omega_Rd_new.zero();
					}

					// Desired angular acceleration in desired frame
					const Vector3f alpha_Rd_raw =
						(omega_Rd_new - _omega_Rd) / dt_sp;

					// Low-pass filter desired angular acceleration
					const float alpha_filter =
						expf(
							-6.28318530718f
							* _omega_d_dot_lpf_cutoff
							* dt_sp);

					_alpha_Rd_filt =
						alpha_filter * _alpha_Rd_filt
						+ (1.f - alpha_filter)
						* alpha_Rd_raw;

					_omega_Rd = omega_Rd_new;

				} else {

					_omega_Rd.zero();
					_alpha_Rd_filt.zero();

					_desired_kinematics_initialized = true;
				}

				_q_d_prev = q_d_new;
				_q_d_prev_timestamp = setpoint_timestamp;

				_q_d = q_d_new;

				_thrust_setpoint =
					Vector3f(
						vehicle_attitude_setpoint.thrust_body);

				_attitude_sp_valid = true;
			}
		}

		// =====================================================
		// Check controller activation
		// =====================================================

		const bool run_smc_att_ctrl =

			_vehicle_control_mode.flag_control_attitude_enabled
			&& _vehicle_control_mode.flag_control_rates_enabled
			&& (_vehicle_status.vehicle_type
			    == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);

		if (!run_smc_att_ctrl
		    || !_attitude_valid
		    || !_attitude_sp_valid) {

			_desired_kinematics_initialized = false;

			_q_d_prev_timestamp = 0;

			_omega_Rd.zero();
			_alpha_Rd_filt.zero();

			perf_end(_loop_perf);

			return;
		}

		// =========================================================
		// Desired angular velocity and acceleration
		//
		// Paper:
		//
		// omega_d = R^T R_d omega_Rd
		//
		// alpha_d =
		// -omega x (R^T R_d omega_Rd)
		// + R^T R_d alpha_Rd
		//
		// =========================================================

		const Dcmf R(_q);
		const Dcmf R_d(_q_d);

		const Matrix3f R_error =
			R.transpose() * R_d;

		const Vector3f omega_d =
			R_error * _omega_Rd;

		const Vector3f omega_dot_d =
			-(omega % omega_d)
			+ R_error * _alpha_Rd_filt;

		// =========================================================
		// Quaternion attitude error
		//
		// PAPER CONVENTION:
		//
		// q_e = q_d^* ⊗ q
		//
		// q_e = [ q_we ; q_e_vec ]
		//
		// IMPORTANT:
		// Do NOT canonicalize q_e here.
		//
		// =========================================================

		const Quatf q_e =
			_q_d.inversed() * _q;

		const float q_we =
			q_e(0);

		const Vector3f q_e_vec =
			q_e.imag();

		// =========================================================
		// sgn_+(q_we)
		//
		//             +1, q_we >= 0
		// sigma =
		//             -1, q_we < 0
		//
		// This term prevents quaternion unwinding.
		// =========================================================

		const float sigma =
			(q_we >= 0.f) ? 1.f : -1.f;

		// =========================================================
		// Angular velocity tracking error
		//
		// omega_e = omega - omega_d
		// =========================================================

		const Vector3f omega_e =
			omega - omega_d;

		// =========================================================
		// Quaternion error-vector derivative
		//
		// q_e_vec_dot =
		// 1/2 * (
		//     q_we * omega_e
		//     + q_e_vec x omega_e
		// )
		//
		// No numerical differentiation of q_e is required.
		// =========================================================

		const Vector3f q_e_vec_dot =
			0.5f *
			(
				q_we * omega_e
				+ (q_e_vec % omega_e)
			);

		// =========================================================
		// Quaternion sliding surface
		//
		// s_q =
		// omega_e
		// + Lambda_q * sgn_+(q_we) * q_e_vec
		//
		// =========================================================

		const Vector3f s_q =
			omega_e
			+ _smc_lambda.emult(
				sigma * q_e_vec);

		// =========================================================
		// Smooth switching function
		//
		// tanh(s_q / phi_q)
		//
		// _smc_boundary is currently used as phi_q.
		//
		// =========================================================

		Vector3f tanh_s{};

		for (int i = 0; i < 3; i++) {

			const float phi_q =
				fmaxf(
					_smc_boundary(i),
					FLT_EPSILON);

			tanh_s(i) =
				tanhf(
					s_q(i) / phi_q);
		}

		// =========================================================
		// QSMC torque law
		//
		// tau =
		//
		// J_hat * omega_dot_d
		//
		// + omega x (J_hat * omega)
		//
		// - J_hat * Lambda_q
		//   * sgn_+(q_we)
		//   * q_e_vec_dot
		//
		// - J_hat * K_q
		//   * tanh(s_q / phi_q)
		//
		// =========================================================

		const Vector3f Iomega =
			_inertia.emult(omega);

		const Vector3f omega_cross_Iomega =
			omega % Iomega;

		const Vector3f quaternion_compensation =
			_smc_lambda.emult(
				sigma * q_e_vec_dot);

		const Vector3f switching_term =
			_smc_k.emult(tanh_s);

		const Vector3f tau_phys =

			_inertia.emult(omega_dot_d)

			+ omega_cross_Iomega

			- _inertia.emult(
				quaternion_compensation)

			- _inertia.emult(
				switching_term);

		// =========================================================
		// Convert physical torque [Nm]
		// to normalized PX4 torque setpoint [-1, 1]
		// =========================================================

		Vector3f torque_setpoint{};

		for (int i = 0; i < 3; i++) {

			const float tau_max =
				fmaxf(
					_torque_max(i),
					FLT_EPSILON);

			torque_setpoint(i) =
				PX4_ISFINITE(tau_phys(i)) ?

				math::constrain(
					tau_phys(i) / tau_max,
					-1.f,
					1.f)

				: 0.f;
		}

		// =========================================================
		// Publish thrust and torque setpoints
		// =========================================================

		vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
		vehicle_torque_setpoint_s vehicle_torque_setpoint{};

		for (int i = 0; i < 3; i++) {

			vehicle_thrust_setpoint.xyz[i] =

				PX4_ISFINITE(
					_thrust_setpoint(i)) ?

				math::constrain(
					_thrust_setpoint(i),
					-1.f,
					1.f)

				: 0.f;

			vehicle_torque_setpoint.xyz[i] =

				PX4_ISFINITE(
					torque_setpoint(i)) ?

				math::constrain(
					torque_setpoint(i),
					-1.f,
					1.f)

				: 0.f;
		}

		vehicle_thrust_setpoint.timestamp_sample =
			angular_velocity.timestamp_sample;

		vehicle_thrust_setpoint.timestamp =
			hrt_absolute_time();

		_vehicle_thrust_setpoint_pub.publish(
			vehicle_thrust_setpoint);

		vehicle_torque_setpoint.timestamp_sample =
			angular_velocity.timestamp_sample;

		vehicle_torque_setpoint.timestamp =
			hrt_absolute_time();

		_vehicle_torque_setpoint_pub.publish(
			vehicle_torque_setpoint);

		updateActuatorControlsStatus(
			vehicle_torque_setpoint,
			dt);
	}

	perf_end(_loop_perf);
}

void
MulticopterAttSmcControl::updateActuatorControlsStatus(
	const vehicle_torque_setpoint_s &vehicle_torque_setpoint,
	float dt)
{
	for (int i = 0; i < 3; i++) {

		_control_energy[i] +=
			vehicle_torque_setpoint.xyz[i]
			* vehicle_torque_setpoint.xyz[i]
			* dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status{};

		status.timestamp =
			vehicle_torque_setpoint.timestamp;

		for (int i = 0; i < 3; i++) {

			status.control_power[i] =
				_control_energy[i]
				/ _energy_integration_time;

			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_pub.publish(status);

		_energy_integration_time = 0.f;
	}
}

int
MulticopterAttSmcControl::task_spawn(
	int argc,
	char *argv[])
{
	bool vtol = false;

	if (argc > 1) {

		if (strcmp(argv[1], "vtol") == 0) {

			vtol = true;
		}
	}

	MulticopterAttSmcControl *instance =
		new MulticopterAttSmcControl(vtol);

	if (instance) {

		_object.store(instance);

		_task_id =
			task_id_is_work_queue;

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
MulticopterAttSmcControl::custom_command(
	int argc,
	char *argv[])
{
	return print_usage("unknown command");
}

int
MulticopterAttSmcControl::print_usage(
	const char *reason)
{
	if (reason) {

		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Quaternion Sliding Mode attitude controller (QSMC)
for multicopter attitude tracking.

Inputs:
- vehicle_attitude
- vehicle_angular_velocity
- vehicle_attitude_setpoint

Outputs:
- vehicle_torque_setpoint
- vehicle_thrust_setpoint

The controller directly replaces the standard cascaded
mc_att_control + mc_rate_control path for QSMC verification.

Quaternion attitude error:

q_e = q_d^* (x) q

Sliding surface:

s_q = omega_e
      + Lambda_q sgn_+(q_we) q_e_vec

The control law is based on an unwinding-free
quaternion sliding mode formulation.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME(
		"mc_att_smc_control",
		"controller");

	PRINT_MODULE_USAGE_COMMAND("start");

	PRINT_MODULE_USAGE_ARG(
		"vtol",
		"VTOL mode",
		true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C"
__EXPORT int mc_att_smc_control_main(
	int argc,
	char *argv[])
{
	return MulticopterAttSmcControl::main(
		       argc,
		       argv);
}