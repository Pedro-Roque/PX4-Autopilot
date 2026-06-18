/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

/*
 * @file attitude_estimator_q_main.cpp
 *
 * Attitude estimator (quaternion based)
 *
 * @author Pedro Roque <roque@caltech.edu>
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include <float.h>

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/world_magnetic_model/geo_mag_declination.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
// Subscribed headers
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
// Published headers
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/estimator_status.h>

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;
using matrix::wrap_pi;

using namespace time_literals;

class ExternalEstimator : public ModuleBase<ExternalEstimator>, public ModuleParams, public px4::WorkItem
{
public:

	ExternalEstimator();
	~ExternalEstimator() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:

	void Run() override;

	void update_vehicle_angular_velocity();

	void update_vehicle_attitude();

	void update_vehicle_local_position();

	void update_vehicle_global_position();

	void publish_estimator_status();

	void publish_vehicle_odometry();

	void update_parameters(bool force = false);

	const float _dt_min = 0.00001f;
	const float _dt_max = 0.02f;

	// WQ tracks vehicle angular velocity, as lowest-level possible for takeover
	uORB::SubscriptionCallbackWorkItem _vehicle_ang_vel_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};

	// publication
	uORB::PublicationData<vehicle_odometry_s> _odometry_pub{ORB_ID(vehicle_odometry)};
	uORB::PublicationData<estimator_status_s> _est_status_pub{ORB_ID(estimator_status)};

	// track timestampsS
	hrt_abstime _latest_ang_vel_timestamp{};
	hrt_abstime _latest_att_timestamp{};
	hrt_abstime _latest_lpos_timestamp{};
	hrt_abstime _latest_gpos_timestamp{};

	vehicle_angular_velocity_s _latest_valid_ang_vel{};
	vehicle_attitude_s _latest_valid_att{};
	vehicle_local_position_s _latest_valid_lpos{};
	vehicle_global_position_s _latest_valid_gpos{};

	float       _received_estimation_timeout{};
	bool        _initialized{false};

	estimator_status_s _status{};

	union solution_status_u {
		struct {
			uint16_t attitude           : 1;
			uint16_t velocity_horiz     : 1;
			uint16_t velocity_vert      : 1;
			uint16_t pos_horiz_rel      : 1;
			uint16_t pos_horiz_abs      : 1;
			uint16_t pos_vert_abs       : 1;
			uint16_t pos_vert_agl       : 1;
			uint16_t const_pos_mode     : 1;
			uint16_t pred_pos_horiz_rel : 1;
			uint16_t pred_pos_horiz_abs : 1;
			uint16_t gps_glitch         : 1;
			uint16_t accel_error        : 1;
		} flags;
		uint16_t value;
	} soln_status{};


	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::EE_TIMEOUT>)       _param_ee_timeout
	)
};

ExternalEstimator::ExternalEstimator() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	update_parameters(true);
}

bool ExternalEstimator::init()
{
	uORB::SubscriptionData<vehicle_angular_velocity_s> vehicle_ang_vel_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::SubscriptionData<vehicle_attitude_s> vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::SubscriptionData<vehicle_local_position_s> vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::SubscriptionData<vehicle_global_position_s> vehicle_global_position_sub{ORB_ID(vehicle_global_position)};

	// check if vehicle_angular_velocity exists
	vehicle_ang_vel_sub.update();
	if (vehicle_ang_vel_sub.advertised() && (hrt_elapsed_time(&vehicle_ang_vel_sub.get().timestamp) < 1_s)) {
		PX4_ERR("init failed, vehicle_angular_velocity already advertised");
		return false;
	}

	vehicle_attitude_sub.update();
	if (vehicle_attitude_sub.advertised() && (hrt_elapsed_time(&vehicle_attitude_sub.get().timestamp) < 1_s)) {
		PX4_ERR("init failed, vehicle_attitude already advertised");
		return false;
	}

	// check if vehicle_angular_velocity exists
	vehicle_local_position_sub.update();
	if (vehicle_local_position_sub.advertised() && (hrt_elapsed_time(&vehicle_local_position_sub.get().timestamp) < 1_s)) {
		PX4_ERR("init failed, vehicle_local_position already advertised");
		return false;
	}

	vehicle_global_position_sub.update();
	if (vehicle_global_position_sub.advertised() && (hrt_elapsed_time(&vehicle_global_position_sub.get().timestamp) < 1_s)) {
		PX4_ERR("init failed, vehicle_global_position already advertised");
		return false;
	}

	if (!_vehicle_ang_vel_sub.registerCallback()) {
		PX4_ERR("_vehicle_ang_vel_sub callback registration failed");
		return false;
	}

	return true;
}

void ExternalEstimator::update_parameters(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		// disable mag fusion if the system does not have a mag
		_received_estimation_timeout = _param_ee_timeout.get();
	}
}

void ExternalEstimator::Run()
{
	if (should_exit()) {
		_vehicle_ang_vel_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_vehicle_ang_vel_sub.updated()) {
		// Get external data
		update_vehicle_angular_velocity();
		update_vehicle_global_position();
		update_vehicle_local_position();
		update_vehicle_attitude();

		// Publish estimator status based on external data
		publish_estimator_status();

		// Published odometry based on external data
		publish_vehicle_odometry();
	}
}

void ExternalEstimator::update_vehicle_angular_velocity()
{
	if (_vehicle_ang_vel_sub.updated()) {
		vehicle_angular_velocity_s _ang_vel;

		if (_vehicle_ang_vel_sub.update(&_ang_vel)) {
			if (hrt_elapsed_time(&_ang_vel.timestamp) < _received_estimation_timeout) {
				_latest_ang_vel_timestamp = _ang_vel.timestamp;
				_latest_valid_ang_vel = _ang_vel;
				if (!_initialized) _initialized = true;
			}
		}
	}
}

void ExternalEstimator::update_vehicle_attitude()
{
	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s _att;

		if (_vehicle_attitude_sub.update(&_att)) {

			if (hrt_elapsed_time(&_att.timestamp) < _received_estimation_timeout) {
				_latest_att_timestamp = _att.timestamp;
				_latest_valid_att = _att;
				soln_status.flags.attitude = 1;
			} else {
				/* position data is outdated - all fields related to lpos should be set to faulty */
				soln_status.flags.attitude = 0;
			}
		}
	}
}

void ExternalEstimator::update_vehicle_local_position()
{
	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s lpos;

		if (_vehicle_local_position_sub.update(&lpos)) {

			if (hrt_elapsed_time(&lpos.timestamp) < _received_estimation_timeout) {
				_latest_lpos_timestamp = lpos.timestamp;
				_latest_valid_lpos = lpos;

				if(lpos.v_xy_valid) {
					soln_status.flags.velocity_horiz = 1;
				}
				if(lpos.v_z_valid) {
					soln_status.flags.velocity_vert = 1;
				}
				if(lpos.xy_valid) {
					soln_status.flags.pos_horiz_rel = 1;
				}
				if(lpos.z_valid) {
					soln_status.flags.pos_vert_agl = 1;
				}
			} else {
				/* position data is outdated - all fields related to lpos should be set to faulty */
				soln_status.flags.pos_horiz_rel = 0;
				soln_status.flags.velocity_horiz = 0;
				soln_status.flags.velocity_vert = 0;
			}
		}
	}
}

void ExternalEstimator::update_vehicle_global_position()
{
	if (_vehicle_global_position_sub.updated()) {
		vehicle_global_position_s gpos;

		if (_vehicle_global_position_sub.update(&gpos)) {

			if (hrt_elapsed_time(&gpos.timestamp) < _received_estimation_timeout) {
				_latest_gpos_timestamp = gpos.timestamp;
				_latest_valid_gpos = gpos;
				if(gpos.lat_lon_valid) {
					soln_status.flags.pos_horiz_abs = 1;
				}
				if(gpos.alt_valid) {
					soln_status.flags.pos_vert_abs = 1;
				}
			} else {
				/* position data is outdated - all fields related to lpos should be set to faulty */
				soln_status.flags.pos_horiz_abs = 0;
				soln_status.flags.pos_vert_abs = 0;
			}
		}
	}
}

void ExternalEstimator::publish_vehicle_odometry()
{
	if (_latest_att_timestamp != 0 && (hrt_absolute_time() - _latest_att_timestamp < _received_estimation_timeout) &&
	    _latest_ang_vel_timestamp != 0 && (hrt_absolute_time() - _latest_ang_vel_timestamp < _received_estimation_timeout) &&
	    _latest_lpos_timestamp != 0 && (hrt_absolute_time() - _latest_lpos_timestamp < _received_estimation_timeout)) {

		// generate vehicle odometry data
		vehicle_odometry_s odom;
		odom.timestamp_sample = _latest_ang_vel_timestamp;

		// position
		odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
		Vector3f(_latest_valid_lpos.x, _latest_valid_lpos.y, _latest_valid_lpos.z).copyTo(odom.position);

		// orientation quaternion
		Quatf(_latest_valid_att.q).copyTo(odom.q);

		// velocity
		odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED;
		Vector3f(_latest_valid_lpos.vx, _latest_valid_lpos.vy, _latest_valid_lpos.vz).copyTo(odom.velocity);

		// angular_velocity
		Vector3f(_latest_valid_ang_vel.xyz[0], _latest_valid_ang_vel.xyz[1], _latest_valid_ang_vel.xyz[2]).copyTo(odom.angular_velocity);

		odom.quality = 0;

		// publish vehicle odometry data
		odom.timestamp = hrt_absolute_time();
		_odometry_pub.publish(odom);
	    }
}

void ExternalEstimator::publish_estimator_status()
{
	if(_initialized){
		_status.timestamp_sample = _latest_ang_vel_timestamp;

		_status.solution_status_flags = soln_status.value;

		_status.timestamp = hrt_absolute_time();
		_est_status_pub.publish(_status);
	}
}


int ExternalEstimator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ExternalEstimator::task_spawn(int argc, char *argv[])
{
	ExternalEstimator *instance = new ExternalEstimator();

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

int ExternalEstimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
External attitude estimator.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ExternalEstimator", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int external_estimator_main(int argc, char *argv[])
{
	return ExternalEstimator::main(argc, argv);
}
