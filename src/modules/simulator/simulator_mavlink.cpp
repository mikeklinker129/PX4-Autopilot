/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
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

#include <px4_log.h>
#include <px4_time.h>
#include "simulator.h"
#include "errno.h"
#include <drivers/drv_pwm_output.h>

using namespace simulator;

#define SEND_INTERVAL 	20
#define UDP_PORT 	14550
#define PIXHAWK_DEVICE "/dev/ttyACM0"

static const uint8_t mavlink_message_lengths[256] = MAVLINK_MESSAGE_LENGTHS;
static const uint8_t mavlink_message_crcs[256] = MAVLINK_MESSAGE_CRCS;

void Simulator::pack_actuator_message(mavlink_hil_controls_t &actuator_msg) {
	float out[8];

	const float pwm_center = (PWM_HIGHEST_MAX + PWM_LOWEST_MIN) / 2;

	// for now we only support quadrotors
	unsigned n = 4;

	for (unsigned i = 0; i < 8; i++) {
		if (_actuators.output[i] > PWM_LOWEST_MIN / 2) {
			if (i < n) {
				// scale PWM out 900..2100 us to 0..1 for rotors */
				out[i] = (_actuators.output[i] - PWM_LOWEST_MIN) / (PWM_HIGHEST_MAX - PWM_LOWEST_MIN);

			} else {
				// scale PWM out 900..2100 us to -1..1 for other channels */
				out[i] = (_actuators.output[i] - pwm_center) / ((PWM_HIGHEST_MAX - PWM_LOWEST_MIN) / 2);
			}

		} else {
			// send 0 when disarmed and for disabled channels */
			out[i] = 0.0f;
		}
	}

	actuator_msg.time_usec = hrt_absolute_time();
	actuator_msg.roll_ailerons = out[0];
	actuator_msg.pitch_elevator = out[1];
	actuator_msg.yaw_rudder = out[2];
	actuator_msg.throttle = out[3];
	actuator_msg.aux1 = out[4];
	actuator_msg.aux2 = out[5];
	actuator_msg.aux3 = out[6];
	actuator_msg.aux4 = out[7];
	actuator_msg.mode = 0; // need to put something here
	actuator_msg.nav_mode = 0;
}

void Simulator::send_data() {
	mavlink_hil_controls_t msg;
	pack_actuator_message(msg);
	send_mavlink_message(MAVLINK_MSG_ID_HIL_CONTROLS, &msg, 200);

	// send heartbeat
	if (hrt_elapsed_time(&_heartbeat_last) >= 1e6) {
		_heartbeat_last = hrt_absolute_time();
		mavlink_heartbeat_t msg;
		msg.base_mode = 0;
		msg.custom_mode = 0;
		msg.autopilot = MAV_AUTOPILOT_PX4;
		msg.mavlink_version = 3;
		send_mavlink_message(MAVLINK_MSG_ID_HEARTBEAT, &msg, 200);
	}

	// send attitude message
	if (hrt_elapsed_time(&_attitude_last) > 20000) {
		_attitude_last = hrt_absolute_time();
		mavlink_attitude_t msg;
		msg.time_boot_ms = _attitude.timestamp / 1000;
		msg.roll = _attitude.roll;
		msg.pitch = _attitude.pitch;
		msg.yaw = _attitude.yaw;
		msg.rollspeed = _attitude.rollspeed;
		msg.pitchspeed = _attitude.pitchspeed;
		msg.yawspeed = _attitude.yawspeed;
		send_mavlink_message(MAVLINK_MSG_ID_ATTITUDE, &msg, 200);
	}

	// send manual control setpoint
	if (hrt_elapsed_time(&_manual_last) > 200000) {
		_manual_last = hrt_absolute_time();
		mavlink_manual_control_t msg;
		msg.x = _manual.x * 1000;
		msg.y = _manual.y * 1000;
		msg.z = _manual.z * 1000;
		msg.r = _manual.r * 1000;
		msg.buttons = 0;
		send_mavlink_message(MAVLINK_MSG_ID_MANUAL_CONTROL, &msg, 200);
	}
}

static void fill_rc_input_msg(struct rc_input_values *rc, mavlink_rc_channels_t *rc_channels) {
	rc->timestamp_publication = hrt_absolute_time();
	rc->timestamp_last_signal = hrt_absolute_time();
	rc->channel_count = rc_channels->chancount;
	rc->rssi = rc_channels->rssi;

	rc->values[0] = rc_channels->chan1_raw;
	rc->values[1] = rc_channels->chan2_raw;
	rc->values[2] = rc_channels->chan3_raw;
	rc->values[3] = rc_channels->chan4_raw;
	rc->values[4] = rc_channels->chan5_raw;
	rc->values[5] = rc_channels->chan6_raw;
	rc->values[6] = rc_channels->chan7_raw;
	rc->values[7] = rc_channels->chan8_raw;
	rc->values[8] = rc_channels->chan9_raw;
	rc->values[9] = rc_channels->chan10_raw;
	rc->values[10] = rc_channels->chan11_raw;
	rc->values[11] = rc_channels->chan12_raw;
	rc->values[12] = rc_channels->chan13_raw;
	rc->values[13] = rc_channels->chan14_raw;
	rc->values[14] = rc_channels->chan15_raw;
	rc->values[15] = rc_channels->chan16_raw;
	rc->values[16] = rc_channels->chan17_raw;
	rc->values[17] = rc_channels->chan18_raw;
}

void Simulator::update_sensors(struct sensor_combined_s *sensor, mavlink_hil_sensor_t *imu) {
	// write sensor data to memory so that drivers can copy data from there
	RawMPUData mpu;
	mpu.accel_x = imu->xacc;
	mpu.accel_y = imu->yacc;
	mpu.accel_z = imu->zacc;
	mpu.temp = imu->temperature;
	mpu.gyro_x = imu->xgyro;
	mpu.gyro_y = imu->ygyro;
	mpu.gyro_z = imu->zgyro;

	write_MPU_data((void *)&mpu);

	RawAccelData accel;
	accel.x = imu->xacc;
	accel.y = imu->yacc;
	accel.z = imu->zacc;

	write_accel_data((void *)&accel);

	RawMagData mag;
	mag.x = imu->xmag;
	mag.y = imu->ymag;
	mag.z = imu->zmag;

	write_mag_data((void *)&mag);

	RawBaroData baro;
	baro.pressure = imu->abs_pressure;
	baro.altitude = imu->pressure_alt;
	baro.temperature = imu->temperature;

	write_baro_data((void *)&baro);
}

void Simulator::handle_message(mavlink_message_t *msg) {
	switch(msg->msgid) {
		case MAVLINK_MSG_ID_HIL_SENSOR:
			mavlink_hil_sensor_t imu;
			mavlink_msg_hil_sensor_decode(msg, &imu);
			update_sensors(&_sensor, &imu);
			break;

		case MAVLINK_MSG_ID_RC_CHANNELS:

			mavlink_rc_channels_t rc_channels;
			mavlink_msg_rc_channels_decode(msg, &rc_channels);
			fill_rc_input_msg(&_rc_input, &rc_channels);

			// publish message
			if(_rc_channels_pub == nullptr) {
				_rc_channels_pub = orb_advertise(ORB_ID(input_rc), &_rc_input);
			} else {
				orb_publish(ORB_ID(input_rc), _rc_channels_pub, &_rc_input);
			}
			break;
	}
}

void Simulator::send_mavlink_message(const uint8_t msgid, const void *msg, uint8_t component_ID) {
	component_ID = 0;
	uint8_t payload_len = mavlink_message_lengths[msgid];
	unsigned packet_len = payload_len + MAVLINK_NUM_NON_PAYLOAD_BYTES;

	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	/* header */
	buf[0] = MAVLINK_STX;
	buf[1] = payload_len;
	/* no idea which numbers should be here*/
	buf[2] = 100;
	buf[3] = 0;
	buf[4] = component_ID;
	buf[5] = msgid;

	/* payload */
	memcpy(&buf[MAVLINK_NUM_HEADER_BYTES],msg, payload_len);

	/* checksum */
	uint16_t checksum;
	crc_init(&checksum);
	crc_accumulate_buffer(&checksum, (const char *) &buf[1], MAVLINK_CORE_HEADER_LEN + payload_len);
	crc_accumulate(mavlink_message_crcs[msgid], &checksum);

	buf[MAVLINK_NUM_HEADER_BYTES + payload_len] = (uint8_t)(checksum & 0xFF);
	buf[MAVLINK_NUM_HEADER_BYTES + payload_len + 1] = (uint8_t)(checksum >> 8);

	ssize_t len = sendto(_fd, buf, packet_len, 0, (struct sockaddr *)&_srcaddr, _addrlen);
	if (len <= 0) {
		PX4_WARN("Failed sending mavlink message");
	}
}

void Simulator::poll_topics() {
	// copy new data if available
	bool updated;
	orb_check(_actuator_outputs_sub, &updated);
	if(updated) {
		orb_copy(ORB_ID(actuator_outputs), _actuator_outputs_sub, &_actuators);
	}

	orb_check(_vehicle_attitude_sub, &updated);
	if(updated) {
		orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_attitude);
	}

	orb_check(_manual_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}
}

void *Simulator::sending_trampoline(void *) {
	_instance->send();
	return 0;	// why do I have to put this???
}

void Simulator::send() {
	px4_pollfd_struct_t fds[1];
	fds[0].fd = _actuator_outputs_sub;
	fds[0].events = POLLIN;

	_time_last = 0;
	_heartbeat_last = 0;
	_attitude_last = 0;
	_manual_last = 0;
	int pret = -1;
	while(pret <= 0) {
		pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 1000);
	}

	while(true) {
		// wait for up to 100ms for data
		pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		//timed out
		if (pret == 0)
			continue;

		// this is undesirable but not much we can do
		if (pret < 0) {
			PX4_WARN("poll error %d, %d", pret, errno);
			// sleep a bit before next try
			usleep(100000);
			continue;
		}

		if (fds[0].revents & POLLIN) {
			// got new data to read, update all topics
			poll_topics();
			send_data();
		}
	}
}

void Simulator::updateSamples()
{
	// udp socket data
	struct sockaddr_in _myaddr;
	const int _port = UDP_PORT;

	struct baro_report baro;
	memset(&baro,0,sizeof(baro));
	baro.pressure = 120000.0f;

	// acceleration report
	struct accel_report accel;
	memset(&accel,0,sizeof(accel));
	accel.z = 9.81f;
	accel.range_m_s2 = 80.0f;

	// gyro report
	struct gyro_report gyro;
	memset(&gyro, 0 ,sizeof(gyro));

	// mag report
	struct mag_report mag;
	memset(&mag, 0 ,sizeof(mag));

	// subscribe to topics
	_actuator_outputs_sub = orb_subscribe(ORB_ID(actuator_outputs));
	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

	// try to setup udp socket for communcation with simulator
	memset((char *)&_myaddr, 0, sizeof(_myaddr));
	_myaddr.sin_family = AF_INET;
	_myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	_myaddr.sin_port = htons(_port);

	if ((_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		PX4_WARN("create socket failed\n");
		return;
	}

	if (bind(_fd, (struct sockaddr *)&_myaddr, sizeof(_myaddr)) < 0) {
		PX4_WARN("bind failed\n");
		return;
	}

	// create a thread for sending data to the simulator
	pthread_t sender_thread;

	// initialize threads
	pthread_attr_t sender_thread_attr;
	pthread_attr_init(&sender_thread_attr);
	pthread_attr_setstacksize(&sender_thread_attr, 1000);

	struct sched_param param;
	(void)pthread_attr_getschedparam(&sender_thread_attr, &param);

	/* low priority */
	param.sched_priority = SCHED_PRIORITY_DEFAULT;
	(void)pthread_attr_setschedparam(&sender_thread_attr, &param);
	pthread_create(&sender_thread, &sender_thread_attr, Simulator::sending_trampoline, NULL);
	pthread_attr_destroy(&sender_thread_attr);

	// setup serial connection to autopilot (used to get manual controls)
	int serial_fd = open(PIXHAWK_DEVICE, O_RDWR);

	if (serial_fd < 0) {
		PX4_WARN("failed to open %s", PIXHAWK_DEVICE);
	}

	// tell the device to stream some messages
	char command[] = "\nsh /etc/init.d/rc.usb\n";
	int w = ::write(serial_fd, command, sizeof(command));

	if (w <= 0) {
		PX4_WARN("failed to send streaming command to %s", PIXHAWK_DEVICE);
	}

	char serial_buf[1024];

	struct pollfd fds[2];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;
	fds[1].fd = serial_fd;
	fds[1].events = POLLIN;

	int len = 0;

	// wait for first data from simulator and respond with first controls
	// this is important for the UDP communication to work
	int pret = -1;
	while (pret <= 0) {
		pret = ::poll(&fds[0], (sizeof(fds[0])/sizeof(fds[0])), 100);
	}

	if (fds[0].revents & POLLIN) {
		len = recvfrom(_fd, _buf, sizeof(_buf), 0, (struct sockaddr *)&_srcaddr, &_addrlen);
		send_data();
	}

	// wait for new mavlink messages to arrive
	while (true) {

		pret = ::poll(&fds[0], (sizeof(fds)/sizeof(fds[0])), 100);

		//timed out
		if (pret == 0)
			continue;

		// this is undesirable but not much we can do
		if (pret < 0) {
			PX4_WARN("poll error %d, %d", pret, errno);
			// sleep a bit before next try
			usleep(100000);
			continue;
		}

		// got data from simulator
		if (fds[0].revents & POLLIN) {
			len = recvfrom(_fd, _buf, sizeof(_buf), 0, (struct sockaddr *)&_srcaddr, &_addrlen);
			if (len > 0) {
				mavlink_message_t msg;
				mavlink_status_t status;
				for (int i = 0; i < len; ++i)
				{
					if (mavlink_parse_char(MAVLINK_COMM_0, _buf[i], &msg, &status))
					{
						// have a message, handle it
						handle_message(&msg);
					}
				}
			}
		}

		// got data from PIXHAWK
		if (fds[1].revents & POLLIN) {
			len = ::read(serial_fd, serial_buf, sizeof(serial_buf));
			if (len > 0) {
				mavlink_message_t msg;
				mavlink_status_t status;
				for (int i = 0; i < len; ++i)
				{
					if (mavlink_parse_char(MAVLINK_COMM_0, serial_buf[i], &msg, &status))
					{
						// have a message, handle it
						handle_message(&msg);
					}
				}
			}
		}
	}
}
