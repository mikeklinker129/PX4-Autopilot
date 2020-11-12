

/**
 * @file rhoman_params.c
 * Parameters for multicopter attitude controller.
 *
 */

/**
 * Roll P gain
 *
 * Roll proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Rhoman Control
 */
PARAM_DEFINE_FLOAT(R_CTRL_ROLL_P, 6.5f);

/**
 * Pitch P gain
 *
 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Rhoman Attitude Control
 */
PARAM_DEFINE_FLOAT(R_CTRL_PITCH_P, 6.5f);

/**
 * Yaw P gain
 *
 * Yaw proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 5
 * @decimal 2
 * @increment 0.1
 * @group Rhoman Attitude Control
 */
PARAM_DEFINE_FLOAT(R_CTRL_YAW_P, 2.8f);


