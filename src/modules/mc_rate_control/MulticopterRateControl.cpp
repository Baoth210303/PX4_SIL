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

#include "MulticopterRateControl.hpp"

#include <float.h>

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

namespace
{

/*
 * Conservative normalized torque limits for the first F450 bring-up.
 *
 * These limits are deliberately separate from _smc_torque_max.
 *
 * _smc_torque_max converts physical torque [Nm] to the normalized
 * PX4 torque command, while these values provide an additional
 * safety clamp on the resulting command.
 */
static constexpr float SMC_OUTPUT_LIMIT_ROLL_PITCH = 0.30f;
static constexpr float SMC_OUTPUT_LIMIT_YAW = 0.20f;

} // namespace


MulticopterRateControl::MulticopterRateControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_vehicle_thrust_setpoint_pub(
		vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint)),
	_vehicle_torque_setpoint_pub(
		vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc) : ORB_ID(vehicle_torque_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type =
		vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();

	_controller_status_pub.advertise();
}


MulticopterRateControl::~MulticopterRateControl()
{
	perf_free(_loop_perf);
}


bool
MulticopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}


void
MulticopterRateControl::parameters_updated()
{
	/*
	 * =========================================================
	 * Stock PX4 rate controller parameters
	 * =========================================================
	 *
	 * The stock controller is intentionally retained as a
	 * fallback whenever the analytical QPD trajectory cannot
	 * safely be used.
	 */

	const Vector3f rate_k{
		_param_mc_rollrate_k.get(),
		_param_mc_pitchrate_k.get(),
		_param_mc_yawrate_k.get()
	};

	_rate_control.setPidGains(
		rate_k.emult(
			Vector3f(
				_param_mc_rollrate_p.get(),
				_param_mc_pitchrate_p.get(),
				_param_mc_yawrate_p.get())),

		rate_k.emult(
			Vector3f(
				_param_mc_rollrate_i.get(),
				_param_mc_pitchrate_i.get(),
				_param_mc_yawrate_i.get())),

		rate_k.emult(
			Vector3f(
				_param_mc_rollrate_d.get(),
				_param_mc_pitchrate_d.get(),
				_param_mc_yawrate_d.get()))
	);

	_rate_control.setIntegratorLimit(
		Vector3f(
			_param_mc_rr_int_lim.get(),
			_param_mc_pr_int_lim.get(),
			_param_mc_yr_int_lim.get())
	);

	_rate_control.setFeedForwardGain(
		Vector3f(
			_param_mc_rollrate_ff.get(),
			_param_mc_pitchrate_ff.get(),
			_param_mc_yawrate_ff.get())
	);

	/*
	 * Manual ACRO rate limits.
	 */
	_acro_rate_max =
		Vector3f(
			radians(_param_mc_acro_r_max.get()),
			radians(_param_mc_acro_p_max.get()),
			radians(_param_mc_acro_y_max.get())
		);

	/*
	 * Stock PX4 yaw torque LPF.
	 *
	 * Used only by the PID fallback path.
	 */
	_output_lpf_yaw.setCutoffFreq(
		_param_mc_yaw_tq_cutoff.get()
	);
}


void
MulticopterRateControl::Run()
{
	if (should_exit()) {

		_vehicle_angular_velocity_sub.unregisterCallback();

		exit_and_cleanup();

		return;
	}

	perf_begin(_loop_perf);

	/*
	 * =========================================================
	 * Parameter updates
	 * =========================================================
	 */

	if (_parameter_update_sub.updated()) {

		parameter_update_s param_update{};

		_parameter_update_sub.copy(
			&param_update
		);

		updateParams();

		parameters_updated();
	}

	/*
	 * =========================================================
	 * Run on fresh angular velocity / gyro samples
	 * =========================================================
	 */

	vehicle_angular_velocity_s angular_velocity{};

	if (_vehicle_angular_velocity_sub.update(
		    &angular_velocity)) {

		const hrt_abstime now =
			angular_velocity.timestamp_sample;

		/*
		 * Guard against too small (< 0.125 ms) and
		 * too large (> 20 ms) rate-loop dt.
		 */
		const float dt =
			math::constrain(
				((now - _last_run) * 1e-6f),
				0.000125f,
				0.02f
			);

		_last_run = now;

		const Vector3f rates{
			angular_velocity.xyz
		};

		const Vector3f angular_accel{
			angular_velocity.xyz_derivative
		};

		/*
		 * Wall-clock time is used for uORB freshness tests.
		 */
		const hrt_abstime time_now =
			hrt_absolute_time();

		/*
		 * =====================================================
		 * PX4 state updates
		 * =====================================================
		 */

		_vehicle_control_mode_sub.update(
			&_vehicle_control_mode
		);

		if (_vehicle_land_detected_sub.updated()) {

			vehicle_land_detected_s
			vehicle_land_detected{};

			if (_vehicle_land_detected_sub.copy(
				    &vehicle_land_detected)) {

				_landed =
					vehicle_land_detected.landed;

				_maybe_landed =
					vehicle_land_detected.maybe_landed;
			}
		}

		_vehicle_status_sub.update(
			&_vehicle_status
		);

		/*
		 * =====================================================
		 * Current attitude
		 * =====================================================
		 *
		 * vehicle_attitude supplies q:
		 *
		 * R = R(q)
		 *
		 * The QPD calculations require current attitude and
		 * current body angular velocity in the same rate-loop
		 * iteration.
		 */

		if (_vehicle_attitude_sub.updated()) {

			vehicle_attitude_s vehicle_attitude{};

			if (_vehicle_attitude_sub.copy(
				    &vehicle_attitude)) {

				Quatf q_candidate{
					vehicle_attitude.q
				};

				const float q_norm =
					q_candidate.norm();

				if (q_candidate.isAllFinite()
				    && PX4_ISFINITE(q_norm)
				    && q_norm > 1e-5f) {

					q_candidate.normalize();

					_q = q_candidate;

					_smc_attitude_valid = true;

					_last_attitude_update =
						vehicle_attitude.timestamp;

				} else {

					_smc_attitude_valid = false;
				}
			}
		}

		/*
		 * =====================================================
		 * Analytical SMC attitude reference
		 * =====================================================
		 *
		 * Generated by mc_att_control:
		 *
		 * q_d
		 *
		 * omega_Rd
		 *
		 * alpha_Rd
		 *
		 * omega_Rd and alpha_Rd are expressed in the desired
		 * body frame B_d.
		 */

		if (_smc_attitude_reference_sub.updated()) {

			smc_attitude_reference_s
			smc_reference{};

			if (_smc_attitude_reference_sub.copy(
				    &smc_reference)) {

				_smc_reference_valid = false;

				_smc_reference_saturated =
					smc_reference.saturated;

				_last_smc_reference_update =
					smc_reference.timestamp;

				if (smc_reference.valid
				    && !smc_reference.saturated) {

					Quatf q_d_candidate{
						smc_reference.q_d
					};

					const Vector3f
					omega_Rd_candidate{
						smc_reference.omega_rd
					};

					const Vector3f
					alpha_Rd_candidate{
						smc_reference.alpha_rd
					};

					const float q_d_norm =
						q_d_candidate.norm();

					if (q_d_candidate.isAllFinite()
					    && PX4_ISFINITE(q_d_norm)
					    && q_d_norm > 1e-5f
					    && omega_Rd_candidate.isAllFinite()
					    && alpha_Rd_candidate.isAllFinite()) {

						q_d_candidate.normalize();

						_q_d =
							q_d_candidate;

						_omega_Rd =
							omega_Rd_candidate;

						_alpha_Rd =
							alpha_Rd_candidate;

						_smc_reference_valid =
							true;
					}
				}
			}
		}

		/*
		 * =====================================================
		 * Freshness checks
		 * =====================================================
		 */

		const bool attitude_fresh =
			_last_attitude_update > 0
			&& time_now >= _last_attitude_update
			&& (time_now - _last_attitude_update)
			   < ATTITUDE_TIMEOUT;

		const bool smc_reference_fresh =
			_last_smc_reference_update > 0
			&& time_now >= _last_smc_reference_update
			&& (time_now - _last_smc_reference_update)
			   < SMC_REFERENCE_TIMEOUT;

		if (!attitude_fresh) {
			_smc_attitude_valid = false;
		}

		if (!smc_reference_fresh) {
			_smc_reference_valid = false;
		}

		/*
		 * =====================================================
		 * Stock PX4 rate-setpoint handling
		 * =====================================================
		 *
		 * Keep this entire path because:
		 *
		 * 1. ACRO/manual mode requires it.
		 *
		 * 2. vehicle_rates_setpoint remains the safe fallback
		 *    reference when QPD is unavailable.
		 *
		 * 3. thrust continues to come through the standard
		 *    PX4 attitude/rate interface.
		 */

		vehicle_rates_setpoint_s
		vehicle_rates_setpoint{};

		if (_vehicle_control_mode.flag_control_manual_enabled
		    && !_vehicle_control_mode.flag_control_attitude_enabled) {

			/*
			 * Manual rate control - ACRO mode.
			 */
			manual_control_setpoint_s
			manual_control_setpoint{};

			if (_manual_control_setpoint_sub.update(
				    &manual_control_setpoint)) {

				const Vector3f man_rate_sp{

					math::superexpo(
						manual_control_setpoint.roll,
						_param_mc_acro_expo.get(),
						_param_mc_acro_supexpo.get()
					),

					math::superexpo(
						-manual_control_setpoint.pitch,
						_param_mc_acro_expo.get(),
						_param_mc_acro_supexpo.get()
					),

					math::superexpo(
						manual_control_setpoint.yaw,
						_param_mc_acro_expo_y.get(),
						_param_mc_acro_supexpoy.get()
					)
				};

				_rates_setpoint =
					man_rate_sp.emult(
						_acro_rate_max
					);

				_thrust_setpoint(2) =
					-(manual_control_setpoint.throttle
					  + 1.f) * .5f;

				_thrust_setpoint(0) = 0.f;
				_thrust_setpoint(1) = 0.f;

				/*
				 * Preserve stock vehicle_rates_setpoint
				 * publication in ACRO.
				 */
				vehicle_rates_setpoint.roll =
					_rates_setpoint(0);

				vehicle_rates_setpoint.pitch =
					_rates_setpoint(1);

				vehicle_rates_setpoint.yaw =
					_rates_setpoint(2);

				_thrust_setpoint.copyTo(
					vehicle_rates_setpoint.thrust_body
				);

				vehicle_rates_setpoint.timestamp =
					hrt_absolute_time();

				_vehicle_rates_setpoint_pub.publish(
					vehicle_rates_setpoint
				);
			}

		} else if (_vehicle_rates_setpoint_sub.update(
				   &vehicle_rates_setpoint)) {

			if (_vehicle_rates_setpoint_sub.copy(
				    &vehicle_rates_setpoint)) {

				_rates_setpoint(0) =
					PX4_ISFINITE(
						vehicle_rates_setpoint.roll
					)
					? vehicle_rates_setpoint.roll
					: rates(0);

				_rates_setpoint(1) =
					PX4_ISFINITE(
						vehicle_rates_setpoint.pitch
					)
					? vehicle_rates_setpoint.pitch
					: rates(1);

				_rates_setpoint(2) =
					PX4_ISFINITE(
						vehicle_rates_setpoint.yaw
					)
					? vehicle_rates_setpoint.yaw
					: rates(2);

				_thrust_setpoint =
					Vector3f(
						vehicle_rates_setpoint.thrust_body
					);
			}
		}

		/*
		 * =====================================================
		 * Rate controller
		 * =====================================================
		 */

		if (_vehicle_control_mode.flag_control_rates_enabled) {

			/*
			 * Stock PX4 integral reset condition.
			 */
			if (!_vehicle_control_mode.flag_armed
			    || _vehicle_status.vehicle_type
			       != vehicle_status_s::
			       VEHICLE_TYPE_ROTARY_WING) {

				_rate_control.resetIntegral();
			}

			/*
			 * =================================================
			 * Control-allocation saturation feedback
			 * =================================================
			 *
			 * Preserve the stock feedback path for the PID
			 * controller. QPD itself does not currently use an
			 * integrator or this anti-windup information.
			 */

			control_allocator_status_s
			control_allocator_status{};

			if (_control_allocator_status_sub.update(
				    &control_allocator_status)) {

				Vector<bool, 3>
				saturation_positive;

				Vector<bool, 3>
				saturation_negative;

				if (!control_allocator_status
				    .torque_setpoint_achieved) {

					for (size_t i = 0;
					     i < 3;
					     i++) {

						if (control_allocator_status
						    .unallocated_torque[i]
						    > FLT_EPSILON) {

							saturation_positive(i) =
								true;

						} else if (
							control_allocator_status
							.unallocated_torque[i]
							< -FLT_EPSILON) {

							saturation_negative(i) =
								true;
						}
					}
				}

				_rate_control.setSaturationStatus(
					saturation_positive,
					saturation_negative
				);
			}

			/*
			 * =================================================
			 * Decide whether Quaternion PD can be used
			 * =================================================
			 *
			 * QPD requires:
			 *
			 * - armed vehicle
			 * - attitude control active
			 * - rate control active
			 * - rotary-wing vehicle
			 * - fresh current attitude
			 * - fresh analytical reference
			 * - no upstream reference saturation
			 *
			 * Otherwise the stock PX4 PID is used immediately.
			 */

			bool use_smc =
				_vehicle_control_mode.flag_armed
				&& _vehicle_control_mode
				   .flag_control_attitude_enabled
				&& _vehicle_status.vehicle_type
				   == vehicle_status_s::
				   VEHICLE_TYPE_ROTARY_WING
				&& _smc_attitude_valid
				&& _smc_reference_valid
				&& !_smc_reference_saturated
				&& attitude_fresh
				&& smc_reference_fresh
				&& rates.isAllFinite();

			Vector3f torque_setpoint{};

			/*
			 * =================================================
			 * Quaternion PD Controller
			 * =================================================
			 */

			if (use_smc) {

				/*
				 * ---------------------------------------------
				 * Desired angular-rate transformation
				 * ---------------------------------------------
				 *
				 * Position/attitude reference supplies omega_Rd
				 * in desired body frame B_d, while the gyro
				 * measurement omega is expressed in current body
				 * frame B. Therefore:
				 *
				 * Q = R^T R_d
				 *
				 * omega_d = Q omega_Rd
				 *
				 * The QPD law does not require omega_dot_d.
				 */

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

				/*
				 * ---------------------------------------------
				 * Quaternion attitude error
				 * ---------------------------------------------
				 *
				 * Paper convention:
				 *
				 * q_e = q_d^{-1} (x) q
				 */

				Quatf q_e =
					_q_d.inversed()
					* _q;

				const float q_e_norm =
					q_e.norm();

				if (!q_e.isAllFinite()
				    || !PX4_ISFINITE(q_e_norm)
				    || q_e_norm < 1e-5f
				    || !omega_d.isAllFinite()) {

					use_smc = false;

				} else {

					q_e.normalize();

					const float q_we =
						q_e(0);

					const Vector3f q_e_vec =
						q_e.imag();

					/*
					 * Unwinding-free quaternion sign:
					 *
					 * sigma = +1, q_we >= 0
					 *
					 * sigma = -1, q_we < 0
					 */
					const float sigma =
						(q_we >= 0.f)
						? 1.f
						: -1.f;

					/*
					 * Angular velocity error:
					 *
					 * omega_e = omega - omega_d
					 */
					const Vector3f omega_e =
						rates
						- omega_d;

					/*
					 * -----------------------------------------
					 * Quaternion PD torque law
					 * -----------------------------------------
					 *
					 * tau =
					 *
					 * - Kd omega_e
					 *
					 * - Kp sigma q_e_vec
					 *
					 * sigma selects the shortest quaternion
					 * representation and avoids unwinding.
					 */
					const Vector3f q_error_shortest =
						sigma
						* q_e_vec;

					const Vector3f tau_phys =
						- _qpd_kd.emult(
							omega_e
						)

						- _qpd_kp.emult(
							q_error_shortest
						);

					/*
					 * -----------------------------------------
					 * Physical torque -> PX4 normalized torque
					 * -----------------------------------------
					 *
					 * u_i =
					 *
					 * tau_i / tau_max_i
					 *
					 * followed by the conservative F450
					 * bring-up output limit.
					 */

					if (tau_phys.isAllFinite()) {

						for (int i = 0;
						     i < 3;
						     i++) {

							const float
							tau_max =
								math::max(
									_smc_torque_max(i),
									FLT_EPSILON
								);

							const float
							output_limit =
								(i < 2)
								? SMC_OUTPUT_LIMIT_ROLL_PITCH
								: SMC_OUTPUT_LIMIT_YAW;

							torque_setpoint(i) =
								math::constrain(
									tau_phys(i)
									/ tau_max,
									-output_limit,
									output_limit
								);
						}

						if (!torque_setpoint
						    .isAllFinite()) {

							use_smc =
								false;
						}

					} else {

						use_smc =
							false;
					}
				}
			}

			/*
			 * =================================================
			 * Stock PID fallback
			 * =================================================
			 *
			 * Any SMC validity failure falls back during this
			 * SAME gyro iteration.
			 */

			if (!use_smc) {

				torque_setpoint =
					_rate_control.update(
						rates,
						_rates_setpoint,
						angular_accel,
						dt,
						_maybe_landed
						|| _landed
					);

				/*
				 * Preserve stock PX4 yaw torque filtering only
				 * for the PID path.
				 */
				torque_setpoint(2) =
					_output_lpf_yaw.update(
						torque_setpoint(2),
						dt
					);

			} else {

				/*
				 * No PID integrator should remain stored while
				 * QPD owns the torque loop. This prevents an
				 * integral kick if fallback is later required.
				 */
				_rate_control.resetIntegral();

				/*
				 * QPD does NOT filter omega_d or its calculated
				 * torque. Keep the stock yaw LPF
				 * state aligned with the present yaw torque so
				 * that a later transition back to PID is smooth.
				 */
				_output_lpf_yaw.reset(
					torque_setpoint(2)
				);
			}

			/*
			 * =================================================
			 * Controller status
			 * =================================================
			 *
			 * The PX4 rate_ctrl_status message represents the
			 * stock PID internals. While QPD is active, publish
			 * a zero-initialized status rather than stale PID
			 * state.
			 */

			rate_ctrl_status_s
			rate_ctrl_status{};

			if (!use_smc) {

				_rate_control.getRateControlStatus(
					rate_ctrl_status
				);
			}

			rate_ctrl_status.timestamp =
				hrt_absolute_time();

			_controller_status_pub.publish(
				rate_ctrl_status
			);

			/*
			 * =================================================
			 * PX4 thrust and torque setpoints
			 * =================================================
			 */

			vehicle_thrust_setpoint_s
			vehicle_thrust_setpoint{};

			vehicle_torque_setpoint_s
			vehicle_torque_setpoint{};

			_thrust_setpoint.copyTo(
				vehicle_thrust_setpoint.xyz
			);

			vehicle_torque_setpoint.xyz[0] =
				PX4_ISFINITE(
					torque_setpoint(0)
				)
				? torque_setpoint(0)
				: 0.f;

			vehicle_torque_setpoint.xyz[1] =
				PX4_ISFINITE(
					torque_setpoint(1)
				)
				? torque_setpoint(1)
				: 0.f;

			vehicle_torque_setpoint.xyz[2] =
				PX4_ISFINITE(
					torque_setpoint(2)
				)
				? torque_setpoint(2)
				: 0.f;

			/*
			 * =================================================
			 * Stock PX4 battery scaling
			 * =================================================
			 */

			if (_param_mc_bat_scale_en.get()) {

				if (_battery_status_sub.updated()) {

					battery_status_s
					battery_status{};

					if (_battery_status_sub.copy(
						    &battery_status)
					    && battery_status.connected
					    && battery_status.scale > 0.f) {

						_battery_status_scale =
							battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.f) {

					for (int i = 0;
					     i < 3;
					     i++) {

						vehicle_thrust_setpoint.xyz[i] =
							math::constrain(
								vehicle_thrust_setpoint.xyz[i]
								* _battery_status_scale,
								-1.f,
								1.f
							);

						vehicle_torque_setpoint.xyz[i] =
							math::constrain(
								vehicle_torque_setpoint.xyz[i]
								* _battery_status_scale,
								-1.f,
								1.f
							);
					}
				}
			}

			/*
			 * =================================================
			 * Re-apply QPD bring-up torque safety clamp
			 * =================================================
			 *
			 * Battery scaling can increase normalized torque.
			 * For the first F450 tests, make sure that scaling
			 * cannot bypass the QPD bring-up limit.
			 */

			if (use_smc) {

				vehicle_torque_setpoint.xyz[0] =
					math::constrain(
						vehicle_torque_setpoint.xyz[0],
						-SMC_OUTPUT_LIMIT_ROLL_PITCH,
						SMC_OUTPUT_LIMIT_ROLL_PITCH
					);

				vehicle_torque_setpoint.xyz[1] =
					math::constrain(
						vehicle_torque_setpoint.xyz[1],
						-SMC_OUTPUT_LIMIT_ROLL_PITCH,
						SMC_OUTPUT_LIMIT_ROLL_PITCH
					);

				vehicle_torque_setpoint.xyz[2] =
					math::constrain(
						vehicle_torque_setpoint.xyz[2],
						-SMC_OUTPUT_LIMIT_YAW,
						SMC_OUTPUT_LIMIT_YAW
					);
			}

			/*
			 * =================================================
			 * Publish stock PX4 actuator setpoint interfaces
			 * =================================================
			 */

			vehicle_thrust_setpoint.timestamp_sample =
				angular_velocity.timestamp_sample;

			vehicle_thrust_setpoint.timestamp =
				hrt_absolute_time();

			_vehicle_thrust_setpoint_pub.publish(
				vehicle_thrust_setpoint
			);

			vehicle_torque_setpoint.timestamp_sample =
				angular_velocity.timestamp_sample;

			vehicle_torque_setpoint.timestamp =
				hrt_absolute_time();

			_vehicle_torque_setpoint_pub.publish(
				vehicle_torque_setpoint
			);

			updateActuatorControlsStatus(
				vehicle_torque_setpoint,
				dt
			);
		}
	}

	perf_end(_loop_perf);
}


void
MulticopterRateControl::updateActuatorControlsStatus(
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

		_actuator_controls_status_pub.publish(
			status
		);

		_energy_integration_time = 0.f;
	}
}


int
MulticopterRateControl::task_spawn(
	int argc,
	char *argv[])
{
	bool vtol = false;

	if (argc > 1) {

		if (strcmp(
			    argv[1],
			    "vtol") == 0) {

			vtol = true;
		}
	}

	MulticopterRateControl *instance =
		new MulticopterRateControl(
			vtol
		);

	if (instance) {

		_object.store(
			instance
		);

		_task_id =
			task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {

		PX4_ERR(
			"alloc failed"
		);
	}

	delete instance;

	_object.store(
		nullptr
	);

	_task_id = -1;

	return PX4_ERROR;
}


int
MulticopterRateControl::custom_command(
	int argc,
	char *argv[])
{
	return print_usage(
		"unknown command"
	);
}


int
MulticopterRateControl::print_usage(
	const char *reason)
{
	if (reason) {

		PX4_WARN(
			"%s\n",
			reason
		);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It retains the stock PX4 rate
PID as a fallback and uses the analytical Quaternion PD Controller
when a fresh and valid SMC attitude reference is available.

The Quaternion PD controller receives q_d and omega_Rd from mc_att_control via
the existing SMC attitude-reference transport and publishes torque through the
standard PX4 vehicle_torque_setpoint interface.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME(
		"mc_rate_control",
		"controller"
	);

	PRINT_MODULE_USAGE_COMMAND(
		"start"
	);

	PRINT_MODULE_USAGE_ARG(
		"vtol",
		"VTOL mode",
		true
	);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


extern "C" __EXPORT int
mc_rate_control_main(
	int argc,
	char *argv[])
{
	return MulticopterRateControl::main(
		argc,
		argv
	);
}