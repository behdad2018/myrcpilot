#include <stdio.h>
#include <unistd.h>
#include <getopt.h>

#include <rc/start_stop.h>
#include <rc/adc.h>
#include <rc/servo.h>
#include <rc/mpu.h>
#include <rc/dsm.h>
#include <rc/bmp.h>
#include <rc/button.h>
#include <rc/led.h>
#include <rc/time.h>
#include <rc/cpu.h>

#include <settings.h> // contains extern settings variable
#include <thrust_map.h>
#include <mix.h>
#include <input_manager.h>
#include <setpoint_manager.h>
#include <state_estimator.h>
#include <log_manager.h>
#include <printf_manager.h>
//B:
#include <xbee_packet_t.h>
#include <rc/encoder.h>
#include <signal.h>

void initialization()
{
	if(rc_encoder_init()<0){
		FAIL("ERROR: failed to initialize encoder\n");
	}
	if(rc_servo_init()<0){
		FAIL("ERROR: failed to initialize escs/servo\n");
	}

	int frequency_hz = 50;
	int wakeup_s = 3;
	double wakeup_val = -0.1;
	int ch = 1;
  	printf("waking ESC up from idle for 3 seconds\n");
    for(int i=0;i<=frequency_hz*wakeup_s;i++){
        if(rc_servo_send_esc_pulse_normalized(ch,wakeup_val)==-1) return -1;
        rc_usleep(1000000/frequency_hz);
    }
    printf("done with wakeup period\n");
}

int main()
{
	initialization()

	// new thread
	// if(rc_pthread_create(&input_manager_thread, &input_manager, NULL,
	// 		SCHED_FIFO, INPUT_MANAGER_PRI)==-1){
	// fprintf(stderr, "ERROR in input_manager_init, failed to start thread\n");
	// return -1;
	// }


	FILE* data=fopen("test1","w")
	usleep(50000);
	printf("cleaning up\n");

	double list_servo[]={500,1000,1500,2000};
	double list_escs[]={0,0.5,0.75,1.0};

	for (int i=1;i<4;i++)
	{
		for (int j=1;j<4;j++)
		{
			int servo_hz = 50;
			int encoder_hz = 400;
			int tmax_s = 60;
			int ch = 1;

			double dt = 1000000/encoder_hz;
			int tmax = encoder_hz*tmax_s;
			int rev[tmax];

		  	printf("Sending esc throttle: %.2f\n", list_escs[j]);
		    for(int t=0;t<=tmax;t++){
		    	if ((t % (encoder_hz / servo_hz)) == 0) {
					rc_servo_send_pulse_us(ch, list_servo[i]);
			        rc_servo_send_esc_pulse_normalized(ch, list_escs[j]);
			    }
				rev[t] = rc_encoder_read(1);
		        rc_usleep(dt);
		    }

			fprintf(data,"%d,",list_servo[i])
			fprintf(data,"%d,",list_escs[j])

			for (int t=0;t<tmax;t++)
			{
				fprintf(data,"%d,",rev)
			}
			fprintf(data,"\n")
			printf("%d\r",rev);

		}
	}

	rc_encoder_cleanup();
	rc_servo_cleanup();
}

