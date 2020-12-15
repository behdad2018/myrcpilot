/**
 * @file feedback.c
 *
 */

#include <stdio.h>
#include <math.h>
#include <rc/math/filter.h>
#include <rc/math/kalman.h>
#include <rc/math/quaternion.h>
#include <rc/math/other.h>
#include <rc/start_stop.h>
#include <rc/led.h>
#include <rc/mpu.h>
#include <rc/servo.h>
#include <rc/time.h>
#include <sensor_calc.h>
#include <feedback.h>
#include <state_estimator.h>
#include <rc_pilot_defs.h>
#include <setpoint_manager.h>
#include <log_manager.h>
#include <settings.h>
#include <mix.h>
#include <thrust_map.h>
#include <xbee_packet_t.h>
#include <trajectory.h>
#include <trajectory2.h>

#define TWO_PI (M_PI*2.0)

int counter = -1;
int counter2 = 0;

feedback_state_t fstate; // extern variable in feedback.h

// keep original controller gains for scaling later
static double D_roll_gain_orig, D_pitch_gain_orig, D_yaw_gain_orig, D_Z_gain_orig, D_X_4_gain_orig, D_Y_4_gain_orig;


// filters
static rc_filter_t D_roll   = RC_FILTER_INITIALIZER;
static rc_filter_t D_pitch  = RC_FILTER_INITIALIZER;
static rc_filter_t D_yaw    = RC_FILTER_INITIALIZER;
static rc_filter_t D_Z      = RC_FILTER_INITIALIZER;
static rc_filter_t D_Xdot_4 = RC_FILTER_INITIALIZER;
static rc_filter_t D_Xdot_6 = RC_FILTER_INITIALIZER;
static rc_filter_t D_X_4    = RC_FILTER_INITIALIZER;
static rc_filter_t D_X_6    = RC_FILTER_INITIALIZER;
static rc_filter_t D_Ydot_4 = RC_FILTER_INITIALIZER;
static rc_filter_t D_Ydot_6 = RC_FILTER_INITIALIZER;
static rc_filter_t D_Y_4    = RC_FILTER_INITIALIZER;
static rc_filter_t D_Y_6    = RC_FILTER_INITIALIZER;


static int __send_motor_stop_pulse(void)
{
	int i;
	if(settings.num_rotors>8){
		printf("ERROR: set_motors_to_idle: too many rotors\n");
		return -1;
	}
	for(i=0;i<settings.num_rotors;i++){
		fstate.m[i] = -0.1;
		rc_servo_send_esc_pulse_normalized(i+1,-0.1);
	}
	return 0;
}

static void __rpy_init(void)
{
	// get controllers from settings

	rc_filter_duplicate(&D_roll,    settings.roll_controller);
	rc_filter_duplicate(&D_pitch,   settings.pitch_controller);
	rc_filter_duplicate(&D_yaw,     settings.yaw_controller);

	#ifdef DEBUG
	printf("ROLL CONTROLLER:\n");
	rc_filter_print(D_roll);
	printf("PITCH CONTROLLER:\n");
	rc_filter_print(D_pitch);
	printf("YAW CONTROLLER:\n");
	rc_filter_print(D_yaw);
	#endif

	// save original gains as we will scale these by battery voltage later
	D_roll_gain_orig = D_roll.gain;
	D_pitch_gain_orig = D_pitch.gain;
	D_yaw_gain_orig = D_yaw.gain;

	// enable saturation. these limits will be changed late but we need to
	// enable now so that soft start can also be enabled
	rc_filter_enable_saturation(&D_roll,    -MAX_ROLL_COMPONENT, MAX_ROLL_COMPONENT);
	rc_filter_enable_saturation(&D_pitch,   -MAX_PITCH_COMPONENT, MAX_PITCH_COMPONENT);
	rc_filter_enable_saturation(&D_yaw, -MAX_YAW_COMPONENT, MAX_YAW_COMPONENT);
	// enable soft start
	rc_filter_enable_soft_start(&D_roll, SOFT_START_SECONDS);
	rc_filter_enable_soft_start(&D_pitch, SOFT_START_SECONDS);
	rc_filter_enable_soft_start(&D_yaw, SOFT_START_SECONDS);
}


int feedback_disarm(void)
{
	fstate.arm_state = DISARMED;
	// set LEDs
	rc_led_set(RC_LED_RED,1);
	rc_led_set(RC_LED_GREEN,0);
	return 0;
}

void feedback_mid_arm()
{
	fstate.arm_state = MID_ARMING;
}

int feedback_arm(void)
{
	if(fstate.arm_state==ARMED){
		printf("WARNING: trying to arm when controller is already armed\n");
		return -1;
	}
	// start a new log file every time controller is armed, this may take some
	// time so do it before touching anything else
	if(settings.enable_logging) log_manager_init();
	// get the current time
	fstate.arm_time_ns = rc_nanos_since_boot();
	// reset the index
	fstate.loop_index = 0;
	// when swapping from direct throttle to altitude control, the altitude
	// controller needs to know the last throttle input for smooth transition
	// TODO: Reinitialize altitude bias
	//last_en_alt_ctrl = 0;
	//last_usr_thr = MIN_Z_COMPONENT;
	// yaw estimator can be zero'd too
	// TODO: Reinitialize yaw estimate
	//num_yaw_spins = 0;
	//last_yaw = -mpu_data.fused_TaitBryan[TB_YAW_Z]; // minus because NED coordinates
	// zero out all filters
	rc_filter_reset(&D_roll);
	rc_filter_reset(&D_pitch);
	rc_filter_reset(&D_yaw);
	rc_filter_reset(&D_Z);

	// prefill filters with current error
	rc_filter_prefill_inputs(&D_roll, -state_estimate.roll);
	rc_filter_prefill_inputs(&D_pitch, -state_estimate.pitch);
	// set LEDs
	rc_led_set(RC_LED_RED,0);
	rc_led_set(RC_LED_GREEN,1);
	// last thing is to flag as armed
	fstate.arm_state = ARMED;
	return 0;
}



int feedback_init(void)
{
	state_estimate.counter = counter;
	double tmp;

	__rpy_init();

	rc_filter_duplicate(&D_Z,   settings.altitude_controller);
	rc_filter_duplicate(&D_Xdot_4,  settings.horiz_vel_ctrl_4dof);
	rc_filter_duplicate(&D_Xdot_6,  settings.horiz_vel_ctrl_6dof);
	rc_filter_duplicate(&D_X_4, settings.horiz_pos_ctrl_4dof);
	rc_filter_duplicate(&D_X_6, settings.horiz_pos_ctrl_6dof);
	rc_filter_duplicate(&D_Ydot_4,  settings.horiz_vel_ctrl_4dof);
	rc_filter_duplicate(&D_Ydot_6,  settings.horiz_vel_ctrl_6dof);
	rc_filter_duplicate(&D_Y_4, settings.horiz_pos_ctrl_4dof);
	rc_filter_duplicate(&D_Y_6, settings.horiz_pos_ctrl_6dof);


	#ifdef DEBUG
	printf("ALTITUDE CONTROLLER:\n");
	rc_filter_print(D_Z);
	#endif

	D_Z_gain_orig = D_Z.gain;
	D_X_4_gain_orig = D_X_4.gain;
	D_Y_4_gain_orig = D_Y_4.gain;


	rc_filter_enable_saturation(&D_Z, -1.0, 1.0);
	rc_filter_enable_soft_start(&D_Z, SOFT_START_SECONDS);
	rc_filter_enable_saturation(&D_X_4, -1.0, 1.0);
	rc_filter_enable_soft_start(&D_X_4, SOFT_START_SECONDS);
	rc_filter_enable_saturation(&D_Y_4, -1.0, 1.0);
	rc_filter_enable_soft_start(&D_Y_4, SOFT_START_SECONDS);

	// make sure everything is disarmed them start the ISR
	feedback_disarm();

	fstate.initialized=1;

	return 0;
}

int feedback_march(void)
{
	int i;
	double tmp, min, max;
	double u[6], mot[8];
	double x[4];
	double y[4];
	double z[4];
	log_entry_t new_log;

	// it  won't redfune here the second time the feedback_march is called
	static int last_en_Z_ctrl = 0;

	// Disarm if rc_state is somehow paused without disarming the controller.
	// This shouldn't happen if other threads are working properly.
	if(rc_get_state()!=RUNNING && fstate.arm_state==ARMED){
		feedback_disarm();
	}

	// check for a tipover
	if(fabs(state_estimate.roll)>TIP_ANGLE || fabs(state_estimate.pitch)>TIP_ANGLE){
		feedback_disarm();
		printf("\n TIPOVER DETECTED \n");
	}

	// if not running or not armed, keep the motors in an idle state
	if(rc_get_state()!=RUNNING || fstate.arm_state==DISARMED){
		__send_motor_stop_pulse();
		return 0;
	}

	// We are about to start marching the individual SISO controllers forward.
	// Start by zeroing out the motors signals then add from there.
	for(i=0;i<8;i++) mot[i] = 0.0;
		for(i=0;i<6;i++) u[i] = 0.0;


	/***************************************************************************
	* Throttle/Altitude Controller
	*
	* If transitioning from direct throttle to altitude control, prefill the
	* filter with current throttle input to make smooth transition. This is also
	* true if taking off for the first time in altitude mode as arm_controller
	* sets up last_en_Z_ctrl and last_usr_thr every time controller arms
	***************************************************************************/
	// run altitude controller if enabled
	// this needs work...
	// we need to:
	//      find hover thrust and correct from there
	//      this code does not work a.t.m.

	// B: making automomation
			if(setpoint.en_XY_pos_ctrl){

	//theta_d qnd phi_d can be foud using equation: THETA=(-1/g) A_psi ^ -1 [p^..h - k_d e^. - k_p e]
	// output must be standard: setpoint.roll

				counter = counter +  1; 

				state_estimate.counter = counter;

				if(counter==0){
				// initial rest condition of the drone
					setpoint.x0=xbeeMsg.x;
					setpoint.y0=xbeeMsg.y;
					setpoint.z0=xbeeMsg.z;
				}

				// Mair circle traj definitions
				float R=3;
				float Vcruise=3;
				float nrev=2;
				float t_takeoff=2;
				float t_delay=2;
				float z_target=-1;
				float f=FEEDBACK_HZ;
				float x;
				float y;
				float z;

				circl_traj(setpoint.x0, setpoint.y0, setpoint.z0, counter, R, Vcruise, nrev, t_takeoff, t_delay, z_target, f, &x, &y, &z);

				// planned trajectory
				setpoint.X=x;
				setpoint.Y=y;
				setpoint.Z=z;

				// current states
				state_estimate.X=xbeeMsg.x;
				state_estimate.Y=xbeeMsg.y;
				state_estimate.Z=xbeeMsg.z;


				// error definitions for x y z
				double pos_err_x;
				double pos_err_y;
				double pos_err_z;

				pos_err_x = setpoint.X - state_estimate.X ;
				pos_err_y = setpoint.Y - state_estimate.Y ;
				pos_err_z = setpoint.Z - state_estimate.Z ;

				float ref_pitch;
				float ref_roll;

				// feedforward controller to obtain reference values
				if (setpoint.en_feedforward){
					ref_pitch=sensor_calc_msmt.theta_ref;
					ref_roll=sensor_calc_msmt.phi_ref;
				}
				else {
					ref_pitch=0;
					ref_roll=0;
				}

				//updating the gains based on current battery voltage
				D_X_4.gain = D_X_4_gain_orig * settings.v_nominal/state_estimate.v_batt_lp;
				D_Y_4.gain = D_Y_4_gain_orig * settings.v_nominal/state_estimate.v_batt_lp;
				// control setpoints
				setpoint.pitch = rc_filter_march(&D_X_4, - pos_err_x)  + ref_pitch;
				setpoint.roll = rc_filter_march(&D_Y_4, pos_err_y)     + ref_roll;
				setpoint.yaw = 0;

				// PID saturation to avoid commanding large angles
				// rc_saturate_double(&setpoint.pitch, -0.08, 0.08);
				// rc_saturate_double(&setpoint.roll, -0.08, 0.08);

			}

			if(setpoint.en_Z_ctrl){
				//B: only first cycle (the first time step after altitude controll is on)
				if(last_en_Z_ctrl == 0){
				//  setpoint.Z = state_estimate.alt_bmp // set altitude setpoint to current altitude
					setpoint.Z = state_estimate.Z;
					rc_filter_reset(&D_Z);   //B reset the filter and reads from json
					tmp = -setpoint.Z_throttle / (cos(state_estimate.roll)*cos(state_estimate.pitch));
					rc_filter_prefill_outputs(&D_Z, tmp);
					last_en_Z_ctrl = 1;
				}
				//B: using feedforward to find reft thrust which is hover throttle
				double hover_throttle;
				if (setpoint.en_feedforward){
					hover_throttle=normalized_hover_thrust;
				}
				else{
					hover_throttle = - 0.6;
				}

				//B: updating the gains based on current battery voltage
				D_Z.gain = D_Z_gain_orig * settings.v_nominal/state_estimate.v_batt_lp;
				
				//tmp = rc_filter_march(&D_Z, -setpoint.Z+state_estimate.alt_bmp); //altitude is positive but +Z is down
		tmp = rc_filter_march(&D_Z, pos_err_z ); //altitude is positive but +Z is down

		u[VEC_Z] =  hover_throttle + tmp / (cos(state_estimate.roll)*cos(state_estimate.pitch));
		rc_saturate_double(&u[VEC_Z], MIN_THRUST_COMPONENT, MAX_THRUST_COMPONENT);
		mix_add_input(u[VEC_Z], VEC_Z, mot);
		last_en_Z_ctrl = 1;
	}
	// else use direct throttle
	else{

		// compensate for tilt
		tmp = setpoint.Z_throttle / (cos(state_estimate.roll)*cos(state_estimate.pitch));
		//printf("throttle: %f\n",tmp);
		rc_saturate_double(&tmp, MIN_THRUST_COMPONENT, MAX_THRUST_COMPONENT);
		u[VEC_Z] = tmp;
		mix_add_input(u[VEC_Z], VEC_Z, mot);
		//B:
		//setpoint.yaw = 0;
	}

	/***************************************************************************
	* Roll Pitch Yaw controllers, only run if enabled
	***************************************************************************/
	// position control
	// use equation 

	if(setpoint.en_rpy_ctrl){
		// Roll
		mix_check_saturation(VEC_ROLL, mot, &min, &max);
		if(max>MAX_ROLL_COMPONENT)  max =  MAX_ROLL_COMPONENT;
		if(min<-MAX_ROLL_COMPONENT) min = -MAX_ROLL_COMPONENT;
		rc_filter_enable_saturation(&D_roll, min, max);
		D_roll.gain = D_roll_gain_orig * settings.v_nominal/state_estimate.v_batt_lp;
		 // B: rc_filter_march is PID controller
		u[VEC_ROLL] = rc_filter_march(&D_roll, setpoint.roll - state_estimate.roll);
		mix_add_input(u[VEC_ROLL], VEC_ROLL, mot);

		// pitch
		mix_check_saturation(VEC_PITCH, mot, &min, &max);
		if(max>MAX_PITCH_COMPONENT)  max =  MAX_PITCH_COMPONENT;
		if(min<-MAX_PITCH_COMPONENT) min = -MAX_PITCH_COMPONENT;
		rc_filter_enable_saturation(&D_pitch, min, max);
		D_pitch.gain = D_pitch_gain_orig * settings.v_nominal/state_estimate.v_batt_lp;
		u[VEC_PITCH] = rc_filter_march(&D_pitch, setpoint.pitch - state_estimate.pitch);
		mix_add_input(u[VEC_PITCH], VEC_PITCH, mot);

		// Yaw
		// if throttle stick is down (waiting to take off) keep yaw setpoint at
		// current heading, otherwide update by yaw rate
		mix_check_saturation(VEC_YAW, mot, &min, &max);
		if(max>MAX_YAW_COMPONENT)  max =  MAX_YAW_COMPONENT;
		if(min<-MAX_YAW_COMPONENT) min = -MAX_YAW_COMPONENT;
		rc_filter_enable_saturation(&D_yaw, min, max);
		D_yaw.gain = D_yaw_gain_orig * settings.v_nominal/state_estimate.v_batt_lp;
		u[VEC_YAW] = rc_filter_march(&D_yaw, setpoint.yaw - state_estimate.yaw);
		mix_add_input(u[VEC_YAW], VEC_YAW, mot);
	}
	// otherwise direct throttle to roll pitch yaw
	else{
		// roll
		mix_check_saturation(VEC_ROLL, mot, &min, &max);
		if(max>MAX_ROLL_COMPONENT)  max =  MAX_ROLL_COMPONENT;
		if(min<-MAX_ROLL_COMPONENT) min = -MAX_ROLL_COMPONENT;
		u[VEC_ROLL] = setpoint.roll_throttle;
		rc_saturate_double(&u[VEC_ROLL], min, max);
		mix_add_input(u[VEC_ROLL], VEC_ROLL, mot);

		// pitch
		mix_check_saturation(VEC_PITCH, mot, &min, &max);
		if(max>MAX_PITCH_COMPONENT)  max =  MAX_PITCH_COMPONENT;
		if(min<-MAX_PITCH_COMPONENT) min = -MAX_PITCH_COMPONENT;
		u[VEC_PITCH] = setpoint.pitch_throttle;
		rc_saturate_double(&u[VEC_PITCH], min, max);
		mix_add_input(u[VEC_PITCH], VEC_PITCH, mot);

		// YAW
		mix_check_saturation(VEC_YAW, mot, &min, &max);
		if(max>MAX_YAW_COMPONENT)  max =  MAX_YAW_COMPONENT;
		if(min<-MAX_YAW_COMPONENT) min = -MAX_YAW_COMPONENT;
		u[VEC_YAW] = setpoint.yaw_throttle;
		rc_saturate_double(&u[VEC_YAW], min, max);
		mix_add_input(u[VEC_YAW], VEC_YAW, mot);
	}

	// for 6dof systems, add X and Y
	if(setpoint.en_6dof){
		// X
		mix_check_saturation(VEC_X, mot, &min, &max);
		if(max>MAX_X_COMPONENT)  max =  MAX_X_COMPONENT;
		if(min<-MAX_X_COMPONENT) min = -MAX_X_COMPONENT;
		u[VEC_X] = setpoint.X_throttle;
		rc_saturate_double(&u[VEC_X], min, max);
		mix_add_input(u[VEC_X], VEC_X, mot);

		// Y
		mix_check_saturation(VEC_Y, mot, &min, &max);
		if(max>MAX_Y_COMPONENT)  max =  MAX_Y_COMPONENT;
		if(min<-MAX_Y_COMPONENT) min = -MAX_Y_COMPONENT;
		u[VEC_Y] = setpoint.Y_throttle;
		rc_saturate_double(&u[VEC_Y], min, max);
		mix_add_input(u[VEC_Y], VEC_Y, mot);
	}

	/***************************************************************************
	* Send ESC motor signals immediately at the end of the control loop
	***************************************************************************/
	for(i=0;i<settings.num_rotors;i++){
		rc_saturate_double(&mot[i], 0.0, 1.0);

		// using Hbem model or simple table lookup
		if (setpoint.en_advancedthrustmap){
			fstate.m[i] = sensor_calc_msmt.throttle[i];
		}
		else{
			fstate.m[i] = map_motor_signal(mot[i]);
		}

		// NO NO NO this undoes all the fancy mixing-based saturation
		// done above, idle should be done with MAX_THRUST_COMPONENT instead
		// rc_saturate_double(&fstate.m[i], MOTOR_IDLE_CMD, 1.0);


		// final saturation just to take care of possible rounding errors
		// this should not change the values and is probably excessive
		rc_saturate_double(&fstate.m[i], 0.0, 1.0);

		// finally send pulses!
		rc_servo_send_esc_pulse_normalized(i+1,fstate.m[i]);
	}

	/***************************************************************************
	* Final cleanup, timing, and indexing
	***************************************************************************/
	// Load control inputs into cstate for viewing by outside threads
	for(i=0;i<6;i++) fstate.u[i]=u[i];
	// keep track of loops since arming
		fstate.loop_index++;
	// log us since arming, mostly for the log
	fstate.last_step_ns = rc_nanos_since_boot();

	return 0;
}


int feedback_cleanup(void)
{
	__send_motor_stop_pulse();
	return 0;
}
