/****************************************************************************
 *
 *   Multicopter quaternion SMC controller.
 *
 *   Desired attitude trajectory:
 *
 *   Position SMC
 *
 *        |
 *        v
 *
 *   R_d, omega_Rd, alpha_Rd
 *
 *        |
 *        v
 *
 *   smc_attitude_reference
 *
 *        |
 *        v
 *
 *   mc_att_smc_control
 *
 *        |
 *        v
 *
 *   omega_d, omega_dot_d
 *
 *        |
 *        v
 *
 *   QSMC torque
 *
 ****************************************************************************/

#include "MulticopterAttSmcControl.hpp"

#include <float.h>
#include <math.h>
#include <string.h>

#include <mathlib/math/Functions.hpp>
#include <mathlib/math/Limits.hpp>


using namespace matrix;


MulticopterAttSmcControl::MulticopterAttSmcControl(
	bool vtol) :

	ModuleParams(
		nullptr),

	WorkItem(
		MODULE_NAME,
		px4::wq_configurations::rate_ctrl),

	_vehicle_thrust_setpoint_pub(

		vtol

		? ORB_ID(
			vehicle_thrust_setpoint_virtual_mc)

		: ORB_ID(
			vehicle_thrust_setpoint)),

	_vehicle_torque_setpoint_pub(

		vtol

		? ORB_ID(
			vehicle_torque_setpoint_virtual_mc)

		: ORB_ID(
			vehicle_torque_setpoint)),

	_loop_perf(

		perf_alloc(

			PC_ELAPSED,

			MODULE_NAME
			": cycle"))
{
	_vehicle_status.vehicle_type =

		vehicle_status_s::
		VEHICLE_TYPE_ROTARY_WING;


	parameters_updated();
}


MulticopterAttSmcControl::
~MulticopterAttSmcControl()
{
	perf_free(
		_loop_perf);
}


bool
MulticopterAttSmcControl::init()
{
	if (!_vehicle_angular_velocity_sub
	    .registerCallback()) {

		PX4_ERR(
			"callback registration failed");

		return false;
	}


	return true;
}


void
MulticopterAttSmcControl::
parameters_updated()
{
	/**
	 * Fixed internal gains for initial
	 * SITL/HIL verification.
	 */
}


Vector3f
MulticopterAttSmcControl::saturation(
	const Vector3f &s) const
{
	Vector3f sat_s{};


	for (int i = 0;
	     i < 3;
	     i++) {

		const float boundary =

			fmaxf(

				_smc_boundary(i),

				FLT_EPSILON);


		sat_s(i) =

			math::constrain(

				s(i)
				/ boundary,

				-1.f,

				1.f);
	}


	return sat_s;
}


void
MulticopterAttSmcControl::Run()
{
	if (should_exit()) {

		_vehicle_angular_velocity_sub
		.unregisterCallback();


		exit_and_cleanup();


		return;
	}


	perf_begin(
		_loop_perf);


	// =========================================================
	// Parameter update
	// =========================================================

	if (_parameter_update_sub.updated()) {

		parameter_update_s
		param_update{};


		_parameter_update_sub.copy(
			&param_update);


		updateParams();


		parameters_updated();
	}


	vehicle_angular_velocity_s
	angular_velocity{};


	// =========================================================
	// Controller runs on every fresh angular-velocity sample
	// =========================================================

	if (_vehicle_angular_velocity_sub.update(
		    &angular_velocity)) {


		const hrt_abstime now =

			angular_velocity
			.timestamp_sample;


		const float dt =

			math::constrain(

				((now
				  - _last_run)

				 * 1e-6f),

				0.000125f,

				0.02f);


		_last_run =
			now;


		// =====================================================
		// Current body angular velocity
		// =====================================================

		const Vector3f omega{

			angular_velocity.xyz
		};


		// =====================================================
		// PX4 control state
		// =====================================================

		_vehicle_control_mode_sub
		.update(
			&_vehicle_control_mode);


		_vehicle_status_sub
		.update(
			&_vehicle_status);


		if (_vehicle_land_detected_sub
		    .updated()) {

			vehicle_land_detected_s
			vehicle_land_detected{};


			if (_vehicle_land_detected_sub
			    .copy(
				    &vehicle_land_detected)) {

				_landed =

					vehicle_land_detected
					.landed;


				_maybe_landed =

					vehicle_land_detected
					.maybe_landed;
			}
		}


		// =====================================================
		// Current attitude q
		//
		// EKF -> vehicle_attitude
		// =====================================================

		if (_vehicle_attitude_sub.updated()) {

			vehicle_attitude_s
			vehicle_attitude{};


			if (_vehicle_attitude_sub.copy(
				    &vehicle_attitude)) {

				_q =

					Quatf(
						vehicle_attitude.q);


				_attitude_valid =

					_q.isAllFinite();
			}
		}


		// =====================================================
		// Desired analytical attitude trajectory
		//
		// Produced at Position SMC update rate.
		// =====================================================

		if (_smc_attitude_reference_sub
		    .updated()) {

			smc_attitude_reference_s
			ref{};


			if (_smc_attitude_reference_sub
			    .copy(
				    &ref)) {

				_q_d =

					Quatf(
						ref.q_d);


				_omega_Rd =

					Vector3f(
						ref.omega_rd);


				_alpha_Rd =

					Vector3f(
						ref.alpha_rd);


				_thrust_setpoint =

					Vector3f(
						ref.thrust_body);


				_reference_analytic_valid =

					ref.analytic_valid;


				_reference_saturated =

					ref.saturated;


				/**
				 * If a hard PX4 safety limiter
				 * invalidated the analytical
				 * derivatives:
				 *
				 * continue q_d tracking
				 *
				 * but disable angular feed-forward.
				 */
				if (!_reference_analytic_valid
				    || _reference_saturated) {

					_omega_Rd.zero();

					_alpha_Rd.zero();
				}


				_attitude_ref_valid =

					ref.valid

					&& _q_d.isAllFinite()

					&& _omega_Rd.isAllFinite()

					&& _alpha_Rd.isAllFinite()

					&& _thrust_setpoint
					.isAllFinite();


				_last_reference_update =

					ref.timestamp;
			}
		}


		// =====================================================
		// Reject stale outer-loop reference
		// =====================================================

		if (_last_reference_update == 0

		    ||

		    (hrt_absolute_time()
		     - _last_reference_update)

		    > REFERENCE_TIMEOUT_US) {

			_attitude_ref_valid =
				false;
		}


		// =====================================================
		// Controller activation
		// =====================================================

		const bool run_smc_att_ctrl =

			_vehicle_control_mode
			.flag_control_attitude_enabled

			&&

			_vehicle_control_mode
			.flag_control_rates_enabled

			&&

			(_vehicle_status.vehicle_type

			 == vehicle_status_s::
			 VEHICLE_TYPE_ROTARY_WING);


		if (!run_smc_att_ctrl

		    || !_attitude_valid

		    || !_attitude_ref_valid) {

			perf_end(
				_loop_perf);


			return;
		}


		// =========================================================
		// Desired trajectory frame transformation
		//
		// Position layer gives:
		//
		// omega_Rd
		// alpha_Rd
		//
		// in desired body frame B_d.
		//
		// Current gyro is in current body frame B.
		//
		// Therefore:
		//
		// Q = R^T R_d
		//
		// omega_d =
		//
		// Q omega_Rd
		//
		// omega_dot_d =
		//
		// Q alpha_Rd
		//
		// - omega x omega_d
		// =========================================================

		const Dcmf R{
			_q
		};


		const Dcmf R_d{
			_q_d
		};


		const Dcmf Q =

			R.transpose()

			* R_d;


		const Vector3f omega_d =

			Q
			* _omega_Rd;


		const Vector3f omega_dot_d =
    		Q * _alpha_Rd
    		- (omega % omega_d);


		// =========================================================
		// Quaternion attitude error
		//
		// Paper convention:
		//
		// q_e =
		//
		// q_d^{-1} otimes q
		// =========================================================

		const Quatf q_e =

			_q_d.inversed()

			* _q;


		const float q_we =
			q_e(0);


		const Vector3f q_e_vec =
			q_e.imag();


		// =========================================================
		// Unwinding-free quaternion sign
		//
		// sigma =
		//
		// +1  if q_we >= 0
		//
		// -1  otherwise
		// =========================================================

		const float sigma =

			(q_we >= 0.f)

			? 1.f

			: -1.f;


		// =========================================================
		// Quaternion attitude-error vector
		//
		// e_q =
		//
		// 2 sigma q_e_vec
		// =========================================================

		const Vector3f e_q =

			2.f

			* sigma

			* q_e_vec;


		// =========================================================
		// Angular velocity error
		//
		// omega_e =
		//
		// omega - omega_d
		// =========================================================

		const Vector3f omega_e =

			omega

			- omega_d;


		// =========================================================
		// Analytical quaternion error derivative
		//
		// Experimental implementation:
		//
		// q_e_dot =
		// 0.5 q_e otimes [omega_e, 0]
		//
		// For:
		//
		// e_q = 2 sigma q_e_vec
		//
		// therefore:
		//
		// e_q_dot =
		//
		// sigma *
		// (
		//   q_we * omega_e
		//   + q_e_vec x omega_e
		// )
		//
		// No numerical differentiation.
		// No LPF.
		// =========================================================

		const Vector3f e_q_dot =

			sigma

			* (q_we * omega_e

			   + (q_e_vec % omega_e));


		// =========================================================
		// Sliding surface
		//
		// q_e = q_d^{-1} q
		//
		// s =
		//
		// omega_e + Lambda e_q
		// =========================================================

		const Vector3f s =

			omega_e

			+ _smc_lambda.emult(
				e_q);


		const Vector3f sat_s =

			saturation(
				s);


		// =========================================================
		// Newton-Euler rigid-body term
		// =========================================================

		const Vector3f Iomega =

			_inertia.emult(
				omega);


		const Vector3f omega_cross_Iomega =

			omega

			% Iomega;


		// =========================================================
		// SMC torque law
		//
		// Since:
		//
		// s =
		//
		// omega
		//
		// - omega_d
		//
		// + Lambda e_q
		//
		//
		// choose:
		//
		// tau =
		//
		// J(
		//   omega_dot_d
		//   - Lambda e_q_dot
		// )
		//
		// + omega x J omega
		//
		// - K s
		//
		// - eta sat(s)
		// =========================================================

		const Vector3f tau_phys =

			_inertia.emult(

				omega_dot_d

				- _smc_lambda.emult(
					e_q_dot))

			+ omega_cross_Iomega

			- _smc_k.emult(
				s)

			- _smc_eta.emult(
				sat_s);


		// =========================================================
		// Physical torque -> PX4 normalized torque
		// =========================================================

		Vector3f torque_setpoint{};


		for (int i = 0;
		     i < 3;
		     i++) {

			const float tau_max =

				fmaxf(

					_torque_max(i),

					FLT_EPSILON);


			torque_setpoint(i) =

				PX4_ISFINITE(
					tau_phys(i))

				? math::constrain(

					tau_phys(i)
					/ tau_max,

					-1.f,

					1.f)

				: 0.f;
		}


		// =========================================================
		// PX4 actuator setpoints
		// =========================================================

		vehicle_thrust_setpoint_s
		vehicle_thrust_setpoint{};


		vehicle_torque_setpoint_s
		vehicle_torque_setpoint{};


		for (int i = 0;
		     i < 3;
		     i++) {

			vehicle_thrust_setpoint.xyz[i] =

				PX4_ISFINITE(
					_thrust_setpoint(i))

				? math::constrain(

					_thrust_setpoint(i),

					-1.f,

					1.f)

				: 0.f;


			vehicle_torque_setpoint.xyz[i] =

				PX4_ISFINITE(
					torque_setpoint(i))

				? math::constrain(

					torque_setpoint(i),

					-1.f,

					1.f)

				: 0.f;
		}


		vehicle_thrust_setpoint
		.timestamp_sample =

			angular_velocity
			.timestamp_sample;


		vehicle_thrust_setpoint
		.timestamp =

			hrt_absolute_time();


		_vehicle_thrust_setpoint_pub
		.publish(
			vehicle_thrust_setpoint);


		vehicle_torque_setpoint
		.timestamp_sample =

			angular_velocity
			.timestamp_sample;


		vehicle_torque_setpoint
		.timestamp =

			hrt_absolute_time();


		_vehicle_torque_setpoint_pub
		.publish(
			vehicle_torque_setpoint);


		updateActuatorControlsStatus(

			vehicle_torque_setpoint,

			dt);
	}


	perf_end(
		_loop_perf);
}


void
MulticopterAttSmcControl::
updateActuatorControlsStatus(
	const vehicle_torque_setpoint_s
	&vehicle_torque_setpoint,
	float dt)
{
	for (int i = 0;
	     i < 3;
	     i++) {

		_control_energy[i] +=

			vehicle_torque_setpoint
			.xyz[i]

			* vehicle_torque_setpoint
			.xyz[i]

			* dt;
	}


	_energy_integration_time +=
		dt;


	if (_energy_integration_time
	    > 500e-3f) {

		actuator_controls_status_s
		status{};


		status.timestamp =

			vehicle_torque_setpoint
			.timestamp;


		for (int i = 0;
		     i < 3;
		     i++) {

			status.control_power[i] =

				_control_energy[i]

				/ _energy_integration_time;


			_control_energy[i] =
				0.f;
		}


		_actuator_controls_status_pub
		.publish(
			status);


		_energy_integration_time =
			0.f;
	}
}


int
MulticopterAttSmcControl::task_spawn(
	int argc,
	char *argv[])
{
	bool vtol =
		false;


	if (argc > 1

	    && strcmp(
		    argv[1],
		    "vtol") == 0) {

		vtol =
			true;
	}


	MulticopterAttSmcControl
	*instance =

		new MulticopterAttSmcControl(
			vtol);


	if (instance) {

		_object.store(
			instance);


		_task_id =
			task_id_is_work_queue;


		if (instance->init()) {

			return PX4_OK;
		}


	} else {

		PX4_ERR(
			"alloc failed");
	}


	delete instance;


	_object.store(
		nullptr);


	_task_id =
		-1;


	return PX4_ERROR;
}


int
MulticopterAttSmcControl::custom_command(
	int argc,
	char *argv[])
{
	return print_usage(
		       "unknown command");
}


int
MulticopterAttSmcControl::print_usage(
	const char *reason)
{
	if (reason) {

		PX4_WARN(
			"%s\n",
			reason);
	}


	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Quaternion sliding mode attitude controller.

Inputs:
- vehicle_attitude
- vehicle_angular_velocity
- smc_attitude_reference

Outputs:
- vehicle_torque_setpoint
- vehicle_thrust_setpoint

The controller directly replaces mc_att_control + mc_rate_control.

Desired omega/alpha are generated analytically by the Position SMC
and mapped into the current body frame at the angular-velocity callback rate.
)DESCR_STR");


	PRINT_MODULE_USAGE_NAME(
		"mc_att_smc_control",
		"controller");


	PRINT_MODULE_USAGE_COMMAND(
		"start");


	PRINT_MODULE_USAGE_ARG(
		"vtol",
		"VTOL mode",
		true);


	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();


	return 0;
}


extern "C"
__EXPORT int
mc_att_smc_control_main(
	int argc,
	char *argv[])
{
	return MulticopterAttSmcControl::main(
		       argc,
		       argv);
}