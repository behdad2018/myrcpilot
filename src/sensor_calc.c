#include <stdio.h>
#include <apriltag_manager.h>
#include <vl53l1x.h>
#include <robotcontrol.h>
#include <rc/start_stop.h>
#include <rc/pthread.h>
#include <rc/time.h>
#include <thread_defs.h>
#include <lcm/lcm.h>
#include <tag_pose_t.h>
#include <state_estimator.h>
#include <settings.h>
#include <setpoint_manager.h>
#include <rc/time.h>
#include "read_from_serial.h"
#include "thrust.h"
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <read_from_serial.h>

Extern var sensor_calc_msmt_t sensor_calc_msmt;

static pthread_t sensor_calc_thread;


float init_read_from_serial()
{

  char port[20] = "/dev/ttyO5"; /* port to connect to */
  speed_t baud = B115200; /* baud rate */
  sensor_calc_msmt.fd = open(port, O_RDWR); /* connect to port */

  /* set the other settings (in this case, 9600 8N1) */
  struct termios settings;
  tcgetattr(fd, &settings);

  cfsetospeed(&settings, baud); /* baud rate */
  settings.c_cflag &= ~PARENB; /* no parity */
  settings.c_cflag &= ~CSTOPB; /* 1 stop bit */
  settings.c_cflag &= ~CSIZE;
  settings.c_cflag |= CS8 | CLOCAL; /* 8 bits */
  settings.c_lflag = ICANON; /* canonical mode */
  settings.c_oflag &= ~OPOST; /* raw output */

  tcsetattr(sensor_calc_msmt.fd, TCSANOW, &settings); /* apply the settings */
  tcflush(sensor_calc_msmt.fd, TCOFLUSH);
    
}

void read_sensor_data() {
  int bytes_read = 1;
  int i=-1;
  char array[1500];

  char *ptr;
  while (bytes_read>0) {
	i=i+1;
	char c = 0;
	bytes_read = read(sensor_calc_msmt.fd, &c, 1);
	array[i]=c;
	if (c == '\n') {
		const char s[] = " ";
		char* token = strtok(array,s);
		int j = -1;
		while (token != NULL) {
			token = strtok(NULL,s);
			j=j+1;
			//printf(" %s\n", token);
			if (j==4){
				sensor_calc_msmt.vel[0]=strtod(token, &ptr);
			}
			else if (j==6){
				sensor_calc_msmt.vel[1]=strtod(token, &ptr);
			}
			else if (j==8){
				sensor_calc_msmt.vel[2]=strtod(token, &ptr);
			}
			else if (j==16){
				sensor_calc_msmt.rho =strtod(token, &ptr);
			}
	    }

	    break;
	}
  }
    
  printf("%c",array);
  printf("%lf\n",wind_data.vel[0]);	
  printf("%lf\n",wind_data.vel[1]);	
  printf("%lf\n",wind_data.vel[2]);
  
}

void* sensor_calc_manager(void* ptr) {
    sensor_calc_msmt.vel[0] = 0;
    sensor_calc_msmt.vel[1] = 0;
    sensor_calc_msmt.vel[2] = 0;
    sensor_calc_msmt.rho = 1;
    
    init_read_from_sensor();
    
    sensor_calc_msmt.initialized = 1;

    printf("Initialized sensor_calc.\n");


    while(rc_get_state()!=EXITING){
        read_sensor_data();
        //calculate_thrust();
        rc_usleep(100);
    }
    close(sensor_calc_msmt.fd);
    return NULL;
}

int sensor_calc_manager_init() {
    sensor_calc_msmt.initialized = 0;
    // start thread
    if(rc_pthread_create(&sensor_calc_manager_thread, &sensor_calc_manager, NULL,
                SCHED_FIFO, SENSOR_CALC_MANAGER_PRI)==-1){
        fprintf(stderr, "ERROR in sensor_calc_manager_init, failed to start thread\n");
        return -1;
    }

    // wait for thread to start
    for(int i=0;i<50;i++){
        if(sensor_calc_msmt.initialized) return 0;
        rc_usleep(50000);
    }
    fprintf(stderr, "ERROR in sensor_calc_manager_init, timeout waiting for thread to start\n");
    return -1;
}

int sensor_calc_manager_cleanup() {

    if(sensor_calc_msmt.initialized==0){
        fprintf(stderr, "WARNING in sensor_calc_manager_cleanup, was never initialized\n");
        return -1;
    }
    // wait for the thread to exit
    if(rc_pthread_timed_join(sensor_calc_manager_thread, NULL, SENSOR_CALC_MANAGER_TOUT)==1){
        fprintf(stderr,"WARNING: in sensor_calc_manager_cleanup, thread join timeout\n");
        return -1;
    }
    return 0;
}
