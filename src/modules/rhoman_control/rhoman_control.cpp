
#include "rhoman_control.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_control_mode.h>


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
	// Subscriptions
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int veh_control_mode_sub    = orb_subscribe(ORB_ID(vehicle_control_mode));

	// Publications
	struct actuator_outputs_s actuator_outputs;
	orb_advert_t outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &actuator_outputs);


	// initialize structures
	struct sensor_combined_s sensor_combined;
	struct vehicle_attitude_s vehicle_attitude;
	struct vehicle_control_mode_s control_mode;
	


	// Main loop clocks off of Sensor Combined message. 
	px4_pollfd_struct_t fds[1];
	fds[0].fd = sensor_combined_sub;
	fds[0].events = POLLIN;

	// initialize parameters
	parameters_update(true);

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
			
			orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor_combined);
			orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &vehicle_attitude);
			orb_copy(ORB_ID(vehicle_control_mode), veh_control_mode_sub, &control_mode);

			if (control_mode.flag_control_newctrl_enabled) {
				actuator_outputs.output[0] = 1900;
				actuator_outputs.output[1] = 1901;
				actuator_outputs.output[2] = 1902;
				actuator_outputs.output[3] = 1903;
				actuator_outputs.noutputs = 4;
				actuator_outputs.timestamp = hrt_absolute_time();

				orb_publish(ORB_ID(actuator_outputs), outputs_pub, &actuator_outputs);
			}

		}

		parameters_update();
	}

	orb_unsubscribe(sensor_combined_sub);
	orb_unsubscribe(vehicle_attitude_sub);
	orb_unsubscribe(veh_control_mode_sub);

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