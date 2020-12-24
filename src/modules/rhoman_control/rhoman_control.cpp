
#include "rhoman_control.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <lib/ecl/geo/geo.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/rhoman_outputs.h>
#include <iostream>

#include "Module0.cpp"
#include "Module1.cpp"
#include "Module2.cpp"
#include "Module4.cpp"

 #define MIN(X,Y)	((X) < (Y) ? (X) : (Y))
 #define MAX(X,Y)	((X) < (Y) ? (Y) : (X))


int RhomanControl::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module
	return 0;
}

int RhomanControl::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}
	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */
	return print_usage("unknown command");
}


int RhomanControl::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("rhoman_control",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

RhomanControl *RhomanControl::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;
		case 'f':
			example_flag = true;
			break;
		case '?':
			error_flag = true;
			break;
		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}
	if (error_flag) {
		return nullptr;
	}
	RhomanControl *instance = new RhomanControl(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

RhomanControl::RhomanControl(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void RhomanControl::run()
{

	PX4_INFO("Rhoman Control Started");

	module0();


	// Subscriptions
	int sensor_combined_sub = 				orb_subscribe(ORB_ID(sensor_combined));
	int veh_control_mode_sub = 				orb_subscribe(ORB_ID(vehicle_control_mode));
	int veh_attitude_sub = 					orb_subscribe(ORB_ID(vehicle_attitude));
	int veh_local_position_sub = 			orb_subscribe(ORB_ID(vehicle_local_position));
	int veh_local_position_setpoint_sub = 	orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	int veh_angular_velocity_sub = 			orb_subscribe(ORB_ID(vehicle_angular_velocity));
	int pos_sp_triplet_sub = 				orb_subscribe(ORB_ID(position_setpoint_triplet));
	// int home_position_sub = 				orb_subscribe(ORB_ID(home_position));
	int manual_control_setpoint_sub =		orb_subscribe(ORB_ID(manual_control_setpoint));
	

	// Publications
	// struct actuator_outputs_s 	actuator_outputs;
	// orb_advert_t outputs_pub = 	orb_advertise(ORB_ID(actuator_outputs), &actuator_outputs);

	struct  rhoman_outputs_s 	rhoman_outputs;
	orb_advert_t rhoman_outputs_pub = 	orb_advertise(ORB_ID(rhoman_outputs), &rhoman_outputs);


	// initialize structures
	struct sensor_combined_s 				sensor_combined;
	struct vehicle_control_mode_s 			control_mode;
	struct vehicle_attitude_s 				veh_attitude;
	struct vehicle_local_position_s 		veh_local_position;
	struct vehicle_local_position_setpoint_s veh_local_position_setpoint;
	struct vehicle_angular_velocity_s 		veh_angular_velocity;
	struct position_setpoint_triplet_s      pos_sp_triplet;
	struct manual_control_setpoint_s   		manual_control_sp;
	// struct home_position_s 					home_position;
	
	memset(&sensor_combined, 0, sizeof(sensor_combined));
	memset(&control_mode, 0, sizeof(control_mode));
	memset(&veh_attitude, 0, sizeof(veh_attitude));
	memset(&veh_local_position, 0, sizeof(veh_local_position));
	memset(&veh_local_position_setpoint, 0, sizeof(veh_local_position_setpoint));
	memset(&veh_angular_velocity, 0, sizeof(veh_angular_velocity));
	memset(&pos_sp_triplet, 0, sizeof(pos_sp_triplet));
	memset(&manual_control_sp, 0, sizeof(manual_control_sp));
	// memset(&home_position, 0, sizeof(home_position));


	// Main loop clocks off of Sensor Combined message. 
	px4_pollfd_struct_t fds[1];
	fds[0].fd = sensor_combined_sub;
	fds[0].events = POLLIN;

	// initialize parameters
	parameters_update(true);

	bool flag_state = false;

	while (!should_exit()) {

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {

			// THIS IS OUR MAIN LOOP HERE. 

			// if (!globallocalconverter_initialized()){
			// 	orb_copy(ORB_ID(home_position), home_position_sub, &home_position);
			// 	if (home_position.valid_alt && home_position.valid_hpos){
			// 		globallocalconverter_init(home_position.lat, home_position.lon, home_position.alt);
			// 	}
		 //    }
			
			orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor_combined);
			orb_copy(ORB_ID(vehicle_control_mode), veh_control_mode_sub, &control_mode);
			orb_copy(ORB_ID(vehicle_attitude), veh_attitude_sub, &veh_attitude);
			orb_copy(ORB_ID(vehicle_local_position), veh_local_position_sub, &veh_local_position);
			orb_copy(ORB_ID(vehicle_local_position_setpoint), veh_local_position_setpoint_sub, &veh_local_position_setpoint );
			orb_copy(ORB_ID(position_setpoint_triplet), pos_sp_triplet_sub, &pos_sp_triplet);
			orb_copy(ORB_ID(vehicle_angular_velocity), veh_angular_velocity_sub, &veh_angular_velocity );
			orb_copy(ORB_ID(position_setpoint_triplet), pos_sp_triplet_sub, &pos_sp_triplet);
			orb_copy(ORB_ID(manual_control_setpoint), manual_control_setpoint_sub, &manual_control_sp);


			module1(&veh_attitude, &veh_angular_velocity, &veh_local_position);
			module2(&pos_sp_triplet);
			module4();

			if (flag_state!=control_mode.flag_control_rhoman_enabled){
				PX4_INFO("Rhoman Flag Switch: %i", !flag_state );
				flag_state =control_mode.flag_control_rhoman_enabled;
			}


			// PX4_INFO("%f ", (double) veh_local_position.vz);

			// PX4_INFO("%f  ", pitchdes);

				// Motor 1: front right
				// motor 2: back left
				// motor 3: front left
				// motor 4: back right

				// Thrusts
				//t0: front right
				//t1: back right
				//t2: back left
				//t3: front left
				// FR-BR-BL-FL
			if (1==1){
				rhoman_outputs.output[0] = MAX(MIN(thrusts_r[0]*300/280.0 + 1200,1800),1200);
				rhoman_outputs.output[1] = MAX(MIN(thrusts_r[2]*300/280.0 + 1200,1800),1200);
				rhoman_outputs.output[2] = MAX(MIN(thrusts_r[3]*300/280.0 + 1200,1800),1200);
				rhoman_outputs.output[3] = MAX(MIN(thrusts_r[1]*300/280.0 + 1200,1800),1200);
				cout << rhoman_outputs.output[0] << " " << rhoman_outputs.output[1] << " " << rhoman_outputs.output[2] << " " <<rhoman_outputs.output[3] <<endl;

				// rhoman_outputs.output[0] = 1501;
				// rhoman_outputs.output[1] = 1502;
				// rhoman_outputs.output[2] = 1503;
				// rhoman_outputs.output[3] = 1504;
				rhoman_outputs.noutputs = 4;
				rhoman_outputs.timestamp = hrt_absolute_time();

				// if (control_mode.flag_control_rhoman_enabled) {	
				orb_publish(ORB_ID(rhoman_outputs), rhoman_outputs_pub, &rhoman_outputs);
			}
			// }


		}

		parameters_update();
	}

	orb_unsubscribe(sensor_combined_sub);
	orb_unsubscribe(veh_control_mode_sub);
	orb_unsubscribe(veh_attitude_sub);
	orb_unsubscribe(veh_local_position_sub);
	orb_unsubscribe(veh_local_position_setpoint_sub);
	orb_unsubscribe(veh_angular_velocity_sub);
	

}


void RhomanControl::execute_parameter_update(){
	PX4_INFO("Parameter Update");
	_param_rc_roll_p.get();
	_param_rc_pitch_p.get();
	_param_rc_yaw_p.get();
}


void RhomanControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();

		execute_parameter_update();
	}
}

int RhomanControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
		### Description
		Rhoman Control System.


		### Implementation
		Section describing the high-level implementation of this module.

		### Examples
		CLI usage example:
		$ module start -f -p 42

		)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int rhoman_control_main(int argc, char *argv[])
{


	return RhomanControl::main(argc, argv);
}