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



Extern var sensor_calc_msmt_t sensor_calc_msmt;

static pthread_t sensor_calc_thread;

void read_sensor_data() {
}

void calculate_thrust() {
}

void* sensor_calc_manager(void* ptr) {
    sensor_calc_msmt.initialized = 1;

    printf("Initialized sensor_calc.\n");


    while(rc_get_state()!=EXITING){
        read_sensor_data();
        calculate_thrust();
        rc_usleep(100);
    }

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
