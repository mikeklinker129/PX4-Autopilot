/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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

/**
 * @file main.c
 *
 * Example implementation of a fixed wing attitude controller. This file is a complete
 * fixed wing controller for manual attitude control or auto waypoint control.
 * There is no need to touch any other system components to extend / modify the
 * complete control architecture.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */


#include <poll.h>

#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/sensor_combined.h>


/* Prototypes */


extern "C" __EXPORT int new_control_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int new_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);



/* Variables */
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */





/* Main Thread */
int new_control_thread_main(int argc, char *argv[])
{

	PX4_INFO("Starting New Control");
	// Example: run the loop synchronized to the sensor_combined topic publication
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	int veh_control_mode_sub    = orb_subscribe(ORB_ID(vehicle_control_mode));

	struct vehicle_control_mode_s control_mode;
	memset(&control_mode, 0, sizeof(control_mode));

	struct vehicle_rates_setpoint_s rate_sp;
	memset(&rate_sp, 0, sizeof(rate_sp));
	orb_advert_t rate_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &rate_sp);


	struct actuator_outputs_s actuator_outputs;
	memset(&actuator_outputs, 0, sizeof(actuator_outputs));
	orb_advert_t outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &actuator_outputs);


	px4_pollfd_struct_t fds[1];
	fds[0].fd = sensor_combined_sub;
	fds[0].events = POLLIN;


	int counter = 0; 

	// initialize parameters
	// parameters_update(true);

	while (!thread_should_exit) {

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

			struct sensor_combined_s sensor_combined;
			orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor_combined);
			// TODO: do something with the data...
			orb_copy(ORB_ID(vehicle_control_mode), veh_control_mode_sub, &control_mode);

			counter++;




			if (control_mode.flag_control_newctrl_enabled){

				rate_sp.roll = 1.0f;
				rate_sp.pitch= 1.0f;
				rate_sp.yaw  = 1.0f;
				rate_sp.thrust_body[2]=0.9f;

				orb_publish(ORB_ID(vehicle_rates_setpoint), rate_sp_pub, &rate_sp);

				actuator_outputs.output[0] = 1900;
				actuator_outputs.output[1] = 1901;
				actuator_outputs.output[2] = 1902;
				actuator_outputs.output[3] = 1903;
				actuator_outputs.noutputs = 4;
				actuator_outputs.timestamp = hrt_absolute_time();

				orb_publish(ORB_ID(actuator_outputs), outputs_pub, &actuator_outputs);


				if (counter%250==0){
					warnx("tick");
				}
			
			}



		}

		// parameters_update();
	}

	orb_unsubscribe(sensor_combined_sub);





    return 1;




	// // /* read arguments */
	// // bool verbose = false;

	// // for (int i = 1; i < argc; i++) {
	// // 	if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
	// // 		verbose = true;
	// // 	}
	// // }

	// /* welcome user (warnx prints a line, including an appended\n, with variable arguments */
	// warnx("[example new control] started");
	// /* initialize parameters, first the handles, then the values */

	// /*
	//  * Declare and safely initialize all structs to zero.
	//  *
	//  * These structs contain the system state and things
	//  * like attitude, position, the current waypoint, etc.
	//  */
	// struct vehicle_attitude_s att;
	// memset(&att, 0, sizeof(att));

	// struct vehicle_global_position_s global_pos;
	// memset(&global_pos, 0, sizeof(global_pos));

	// struct vehicle_control_mode_s control_mode;
	// memset(&control_mode, 0, sizeof(control_mode));

	// /* output structs - this is what is sent to the mixer */
	// struct actuator_controls_s actuators;
	// memset(&actuators, 0, sizeof(actuators));

	// bool controller_active = false; 


	// /* publish actuator controls with zero values */
	// for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROLS; i++) {
	// 	actuators.control[i] = 0.0f;
	// }

	// /*
	//  * Advertise that this controller will publish actuator
	//  * control values and the rate setpoint
	//  */
	// // orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);

	// /* subscribe to topics. */
	// int att_sub 				= orb_subscribe(ORB_ID(vehicle_attitude));
	// int global_pos_sub 			= orb_subscribe(ORB_ID(vehicle_global_position));
	// int veh_control_mode_sub    = orb_subscribe(ORB_ID(vehicle_control_mode));



	// // int vehicle_control_mode 	= orb_subscribe(ORB_ID(vehicle_control_mode))


	// //advertise to actuator_control topic 
	// struct actuator_controls_s act;
	// memset(&act, 0, sizeof(act));
	// orb_advert_t act_pub = orb_advertise(ORB_ID(actuator_controls_0), &act);


	// struct vehicle_rates_setpoint_s rate_sp;
	// memset(&rate_sp, 0, sizeof(rate_sp));
	// orb_advert_t rate_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &rate_sp);


	// /* Setup of loop */

	// // struct pollfd fds[1] {};
	// // fds[0].fd = att_sub;
	// // fds[0].events = POLLIN;

	// int tick_counter=0;

	// while (!thread_should_exit) {

		
	// 	 * Wait for a sensor or param update, check for exit condition every 500 ms.
	// 	 * This means that the execution will block here without consuming any resources,
	// 	 * but will continue to execute the very moment a new attitude measurement or
	// 	 * a param update is published. So no latency in contrast to the polling
	// 	 * design pattern (do not confuse the poll() system call with polling).
	// 	 *
	// 	 * This design pattern makes the controller also agnostic of the attitude
	// 	 * update speed - it runs as fast as the attitude updates with minimal latency.
		 
	// 	int ret = poll(fds, 1, 500);

		

	// 	if (ret < 0) {
	// 		/*
	// 		 * Poll error, this will not really happen in practice,
	// 		 * but its good design practice to make output an error message.
	// 		 */
	// 		warnx("poll error");

	// 	} else if (ret == 0) {
	// 		/* no return value = nothing changed for 500 ms, ignore */
	// 	} else {

			
	// 		// // check for parameter updates
	// 		// if (parameter_update_sub.updated()) {
	// 		// 	// clear update
	// 		// 	parameter_update_s pupdate;
	// 		// 	parameter_update_sub.copy(&pupdate);

	// 		// 	// if a param update occured, re-read our parameters
	// 		// 	parameters_update(&ph, &p);
	// 		// }

	// 		/* only run controller if attitude changed */
	// 		if (fds[0].revents & POLLIN) {


	// 			tick_counter++;

	// 			if (tick_counter%250==0){
	// 				warnx("tick");
	// 			}


	// 			if (tick_counter>25000000){
	// 				tick_counter=0;
	// 			}

	// 			/* Check if there is a new position measurement or position setpoint */
	// 			bool pos_updated;
	// 			orb_check(global_pos_sub, &pos_updated);

	// 			/* get a local copy of attitude */
	// 			orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);

	// 			orb_copy(ORB_ID(vehicle_control_mode), veh_control_mode_sub, &control_mode);

	// 			if (control_mode.flag_control_newctrl_enabled != controller_active){

	// 				warnx("WE HIT THE FLAG");

	// 				controller_active = control_mode.flag_control_newctrl_enabled;

	// 			}

	// 			if (controller_active){

	// 			}


	// 			if (control_mode.flag_control_newctrl_enabled){

	// 				if(false){


	// 				act.control[0] = 0.0f;      // roll
	// 				act.control[1] = 0.0f;      // pitch
	// 				act.control[2] = 1.0f;		// yaw
	// 				act.control[3] = 1.0f;		// thrust
	// 				orb_publish(ORB_ID(actuator_controls_0), act_pub, &act);
	// 				}

	// 				rate_sp.roll = 0.0f;
	// 				rate_sp.pitch= 0.0f;
	// 				rate_sp.yaw  = 0.75f;

	// 				orb_publish(ORB_ID(vehicle_rates_setpoint), rate_sp_pub, &rate_sp);
	// 			}


	// 			/* sanity check and publish actuator outputs */
	// 			// if (PX4_ISFINITE(actuators.control[0]) &&
	// 			//     PX4_ISFINITE(actuators.control[1]) &&
	// 			//     PX4_ISFINITE(actuators.control[2]) &&
	// 			//     PX4_ISFINITE(actuators.control[3])) {
	// 			// 	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

	// 			// 	if (verbose) {
	// 			// 		warnx("published");
	// 			// 	}
	// 			// }
	// 		}
	// 	}
	// }

	// printf("[new_control] exiting, stopping all motors.\n");
	// thread_running = false;

	// /* kill all outputs */
	// // for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROLS; i++) {
	// // 	actuators.control[i] = 0.0f;
	// // }

	// // orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

	// fflush(stdout);

	// return 0;
}

/* Startup Functions */

static void
usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: new_control {start|stop|status}\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to px4_task_spawn_cmd().
 */
int new_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("new_control already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		deamon_task = px4_task_spawn_cmd("new_control",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 20,
						 2048,
						 new_control_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);
		thread_running = true;
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tnew_control is running\n");

		} else {
			printf("\tnew_control not started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 0;
}
