#include <sensor_calc.h>
#include <stdio.h>
#include <vl53l1x.h>
#include <robotcontrol.h>
#include <rc/start_stop.h>
#include <rc/pthread.h>
#include <rc/time.h>
#include <thread_defs.h>
#include <rc/time.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <thrust.h>
#include <feedforward.h>
#include <rpmtothrottle.h>
#include <feedback.h>
#include <settings.h>
#include <errno.h> 

int i;

// Definition of our global sensor_calc_msmt variable.
sensor_calc_msmt_t sensor_calc_msmt;

static pthread_t sensor_calc_manager_thread;


float init_read_from_serial()
{

//------------------------------- Opening the Serial Port -------------------------------
    sensor_calc_msmt.fd = open("/dev/ttyO5",O_RDWR | O_NOCTTY);    // ttyUSB0 is the FT232 based USB2SERIAL Converter 
    if(sensor_calc_msmt.fd == -1)                        // Error Checking 
    printf("Error while opening the device\n");
//---------- Setting the Attributes of the serial port using termios structure ---------
    struct termios SerialPortSettings;  // Create the structure                          
    tcgetattr(sensor_calc_msmt.fd, &SerialPortSettings); // Get the current attributes of the Serial port
// Setting the Baud rate
    cfsetispeed(&SerialPortSettings,B115200); // Set Read  Speed as 19200                       
                       

    SerialPortSettings.c_cflag &= ~PARENB;   // Disables the Parity Enable bit(PARENB),So No Parity   
    SerialPortSettings.c_cflag &= ~CSTOPB;   // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit 
    SerialPortSettings.c_cflag &= ~CSIZE;    // Clears the mask for setting the data size             
    SerialPortSettings.c_cflag |=  CS8;      // Set the data bits = 8                                 
    SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control                         
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver,Ignore Modem Control lines        
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);  // Disable XON/XOFF flow control both i/p and o/p 
    SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode 
    SerialPortSettings.c_oflag &= ~OPOST;//No Output Processing
// Setting Time outs 
    SerialPortSettings.c_cc[VMIN] = 13; // Read at least 10 characters 
    SerialPortSettings.c_cc[VTIME] = 0; // Wait indefinetly  

    if((tcsetattr(sensor_calc_msmt.fd,TCSANOW,&SerialPortSettings)) != 0) // Set the attributes to the termios structure
    printf("Error while setting attributes \n");
    //------------------------------- Read data from serial port -----------------------------

    tcflush(sensor_calc_msmt.fd, TCIFLUSH);   // Discards old data in the rx buffer   


	// old code
	// char port[20] = "/dev/ttyO5"; /* port to connect to */
	// speed_t baud = B115200; /* baud rate */
	// sensor_calc_msmt.fd = open(port, O_RDWR); /* connect to port */

	// if(sensor_calc_msmt.fd== -1)                        /* Error Checking */
 //               printf("\n  Error! in Opening ttyO5  ");
 //        else
 //               printf("\n  ttyO5 Opened Successfully ");      

	// /* set the other settings (in this case, 9600 8N1) */
	// struct termios settings;
	// tcgetattr(sensor_calc_msmt.fd, &settings);

	// cfsetospeed(&settings, baud); /* baud rate */
	// settings.c_cflag &= ~PARENB; /* no parity */
	// settings.c_cflag &= ~CSTOPB; /* 1 stop bit */
	// settings.c_cflag &= ~CSIZE;
	// settings.c_cflag |= CS8; /* 8 bits */
	// settings.c_lflag = ICANON; /* canonical mode */
	// settings.c_oflag &= ~OPOST; /* raw output */

	// tcsetattr(sensor_calc_msmt.fd, TCSANOW, &settings); /* apply the settings */
	// tcflush(sensor_calc_msmt.fd, TCOFLUSH);

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
		//printf("%c",c);
		array[i]=c;
		if (c == '\n') {
			const char s[] = " ";
			char* token = strtok(array,s);
			int j = -1;
			while (token != NULL && j<17) {
				token = strtok(NULL,s);
				j=j+1;
			//printf(" %s\n", token);
				// wind sensor, RC Pilot and BeagleBone all have different coordinates definitions!
				// wind sensor need to be in this orieantation so that the following velocities are correct
				// N of wind sensor point to pos Y axis of BeagleBone which is pos X axis in RC pilot 
				// In RC pilot nose in pos X, right wing in pos Y and down in pos Z
				
				// Vx in wind sensor ~ Vy in RC pilot
				if (j==4){
					sensor_calc_msmt.vel[1]=strtod(token, &ptr)+ 0.001F;
				}
				// Vy in wind sensor ~ Vx in RC pilot
				else if (j==6){
					sensor_calc_msmt.vel[0]=strtod(token, &ptr)+ 0.001F;
				}
				// Vz in wind sensor ~ -Vz in RC pilot
				else if (j==8){
					sensor_calc_msmt.vel[2]=-strtod(token, &ptr)+ 0.001F;
				}
				// we want to only read the density at first. why? because sometimes density is retruned zero by the sensor.
				else if (j==16 && sensor_calc_msmt.rho==1.0F){
					sensor_calc_msmt.rho =strtod(token, &ptr);
				}
			}

			break;
		}
	}
	// printf("%c",array);
	// printf("%lf\n",sensor_calc_msmt.vel[0]); 
	// printf("%lf\n",sensor_calc_msmt.vel[1]); 
	// printf("%lf\n",sensor_calc_msmt.vel[2]);
}


void calculate_feedforward(){
	float T_ref;
	float phi_ref;
	float theta_ref;
	feedforward(sensor_calc_msmt.vel,sensor_calc_msmt.rho,&T_ref,&phi_ref,&theta_ref);
	sensor_calc_msmt.T_ref = -T_ref;
	sensor_calc_msmt.normalized_hover_thrust = 0.25 * sensor_calc_msmt.T_ref/sensor_calc_msmt.Tmax;
	sensor_calc_msmt.phi_ref = phi_ref;
	sensor_calc_msmt.theta_ref = theta_ref;
	//printf("T_ref is %lf\n",sensor_calc_msmt.T_ref);
	//printf("phi_ref is %lf\n",sensor_calc_msmt.phi_ref);
	//printf("theta_ref is %lf\n",sensor_calc_msmt.theta_ref);
}

void calculate_rpm() {
		// initial lower and upper bands
	float lb[] = {100.F, 100.F, 100.F, 100.F};
	float ub[] = {12000.F, 12000.F, 12000.F, 12000.F};
	 // finding maxim thrust
	float ignore;
	float Tmax;
	float rpm;
	// RPM value for max thrust does not matter let's say 4000 rpm 
	rpmtothrottle(4000.F,sensor_calc_msmt.vel,&ignore,&Tmax);
	sensor_calc_msmt.Tmax=Tmax+00000.1F;
    //printf("max thrust is %lf\n",sensor_calc_msmt.Tmax);
    //printf("tmax is %lf\n",sensor_calc_msmt.Tmax);
    //printf("thrust is %lf\n",fstate.mot[0]*sensor_calc_msmt.Tmax);
	for(i=0;i<settings.num_rotors;i++){
		
		
		// if (sensor_calc_msmt.rpm[i] > 0) { // If rpm has already been calculated once.
		// 	lb[i] = sensor_calc_msmt.rpm[i] - 500;
		// 	ub[i] = sensor_calc_msmt.rpm[i] + 500;
		// 	if (lb[i] < 0) {
		// 		lb[i]=100;
		// 	}
		// }
		
		thrust(lb[i],ub[i],fstate.mot[i]*sensor_calc_msmt.Tmax,sensor_calc_msmt.rho,sensor_calc_msmt.vel,&rpm);
		sensor_calc_msmt.rpm[i]=rpm;
		//fstate.mot[i]*sensor_calc_msmt.Tmax
		//printf("input thrust is %lf\n",fstate.mot[i]*sensor_calc_msmt.Tmax);
		// printf("RPM is %lf\n",sensor_calc_msmt.rpm[i]);
		// printf("rho is %lf\n",sensor_calc_msmt.rho);
		// printf("v is %lf\n",sensor_calc_msmt.vel[1]);
	}
}

void calculate_throttle(){
	float throttle;
	float ignore2;
	//printf("RPM is %lf\n",sensor_calc_msmt.rpm[0]);
	for(i=0;i<settings.num_rotors;i++){
		
		rpmtothrottle(sensor_calc_msmt.rpm[i],sensor_calc_msmt.vel,&throttle,&ignore2);
		sensor_calc_msmt.throttle[i] = throttle;

		//printf("throttle is %lf\n",sensor_calc_msmt.throttle[i]);
	}
}

void* sensor_calc_manager(void* ptr) {
	sensor_calc_msmt.vel[0] = 0.0F;
	sensor_calc_msmt.vel[1] = 0.0F;
	sensor_calc_msmt.vel[2] = 0.0F;
	sensor_calc_msmt.rho = 1.0F;
	
	init_read_from_serial();

	
	sensor_calc_msmt.initialized = 1;

//	printf("Initialized sensor_calc.\n");


	while(rc_get_state()!=EXITING){
		read_sensor_data();
	    calculate_rpm();
		calculate_throttle();
		calculate_feedforward();
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


