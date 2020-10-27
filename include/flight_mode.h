/**
 * <flight_mode.h>
 */

#ifndef FLIGHT_MODE_H
#define FLIGHT_MODE_H
#include <stdbool.h>
/**
 * This is how the user interacts with the setpoint manager.
 */
typedef enum flight_mode_t{
	/**
	 * test_bench mode does no feedback at all, it takes the raw user inputs
	 * and directly outputs to the motors. This could technically fly but
	 * would not be easy! Designed for confirming mixing matrix and motors
	 * are working. maps Z,Roll,Pitch,Yaw
	 */
	TEST_BENCH_4DOF,
	/**
	 * test_bench mode does no feedback at all, it takes the raw user inputs
	 * and directly outputs to the motors. This could technically fly but
	 * would not be easy! Designed for confirming mixing matrix and motors
	 * are working. maps X,Y,Z,Yaw
	 */
	TEST_BENCH_6DOF,
	/**
	 * user inputs translate directly to the throttle, roll, pitch, & yaw
	 * setpoints. No altitude feedback control. On 6DOF platforms the X & Y
	 * thrust directions are left at 0.
	 */
	DIRECT_THROTTLE_4DOF,
	/**
	 * user inputs translate directly to the throttle, roll, pitch, & yaw
	 * setpoints. No altitude feedback control.Roll and pitch are left at 0
	 */
	DIRECT_THROTTLE_6DOF,
	/**
	 * like DIRECT_THROTTLE for roll/pitch/yaw but feedback is performed to
	 * hold altitude setpoint which is them moved up and down steadily based
	 * on user input.
	 */
	ALT_HOLD_4DOF,
	/**
	 * like DIRECT_THROTTLE for roll/pitch/yaw but feedback is performed to
	 * hold altitude setpoint which is them moved up and down steadily based
	 * on user input.
	 */
	ALT_HOLD_6DOF,
	/**
	 * Control sticks translate to velocity setpoints in horizontal
	 * translation X and Y. Yaw and Altitude are still position setpoints
	 * like alt_hold
	 */
	VELOCITY_CONTROL_4DOF,
	/**
	 * Control sticks translate to velocity setpoints in horizontal
	 * translation X and Y. Yaw and Altitude are still position setpoints
	 * like alt_hold
	 */
	VELOCITY_CONTROL_6DOF,
	/**
	 * PG: TODO: What do you intend for this mode?
	 */
	POSITION_CONTROL_4DOF,
	/**
	 * PG: TODO: What do you intend for this mode?
	 */
	POSITION_CONTROL_6DOF,

	AUTONOMOUS,

	/**
     * Commands 0 roll, 0 pitch, and a fixed throttle to hover or slowely descend
     * Useful as an emergency mode if MOCAP drops out for too long
      */

	OPEN_LOOP_DESCENT


} flight_mode_t;




#endif