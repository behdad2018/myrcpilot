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



#define MAX_ANGLE (4*3.14159/180)

// Extern var.
apriltag_msmt_t apriltag_msmt;

static pthread_t apriltag_manager_thread;

void on_tag_pose(const lcm_recv_buf_t *rbuf, const char* channel, const tag_pose_t* msg, void* user) {
    if (msg->detected) {
        rc_vector_t tb = RC_VECTOR_INITIALIZER;
        rc_vector_t quat = RC_VECTOR_INITIALIZER;
        rc_vector_zeros(&tb, 3);

        tb.d[1] = -state_estimate.tb_imu[1];
        tb.d[0] = -state_estimate.tb_imu[0];
        //tb.d[2] = 3.14159/2;

        double pos[3] = {msg->pos[1], -msg->pos[0], -msg->pos[2]};
        //apriltag_msmt.fx = pos[0];
        //apriltag_msmt.fy = pos[1];
        //apriltag_msmt.fz = pos[2];

        rc_quaternion_from_tb(tb, &quat);
        rc_quaternion_rotate_vector_array(pos, quat.d);

        double vx = (pos[0] - apriltag_msmt.x)/(1e-9 * (rc_nanos_since_boot() - apriltag_msmt.ntime));
        apriltag_msmt.x = pos[0];

        double vy = (pos[1] - apriltag_msmt.y)/(1e-9 * (rc_nanos_since_boot() - apriltag_msmt.ntime));
        apriltag_msmt.y = pos[1];

        pos[2] += 0.022;
        double vz = (pos[2] - apriltag_msmt.z)/(1e-9 * (rc_nanos_since_boot() - apriltag_msmt.ntime));
        apriltag_msmt.z = pos[2];

        if (!apriltag_msmt.detected) {
            vx = 0;
            vy = 0;
            vz = 0;
        }

        double alpha = 0.9;
        apriltag_msmt.vx = alpha*vx + (1-alpha)*apriltag_msmt.vx;
        apriltag_msmt.vy = alpha*vy + (1-alpha)*apriltag_msmt.vy;
        apriltag_msmt.vz = alpha*vz + (1-alpha)*apriltag_msmt.vz;

        double fx = settings.at_kp*(apriltag_msmt.x - setpoint.X) + settings.at_kd*apriltag_msmt.vx;
        if (fx > MAX_ANGLE) {
            fx = MAX_ANGLE;
        }
        if (fx < -MAX_ANGLE) {
            fx = -MAX_ANGLE;
        }
        apriltag_msmt.fx = fx;
        double fy = -settings.at_kp*(apriltag_msmt.y - setpoint.Y) + settings.at_kd*apriltag_msmt.vy;
        if (fy > MAX_ANGLE) {
            fy = MAX_ANGLE;
        }
        if (fy < -MAX_ANGLE) {
            fy = -MAX_ANGLE;
        }
        apriltag_msmt.fy = fy;
        double fz = settings.at_kp*apriltag_msmt.z + settings.at_kd*apriltag_msmt.vz;
        if (fz > MAX_ANGLE) {
            fz = MAX_ANGLE;
        }
        if (fz < -MAX_ANGLE) {
            fz = -MAX_ANGLE;
        }
        apriltag_msmt.fz = fz;
    } else {
        apriltag_msmt.fx = 0;
        apriltag_msmt.fy = 0;
        apriltag_msmt.fz = 0;
    }

    apriltag_msmt.detected = msg->detected;
    apriltag_msmt.ntime = rc_nanos_since_boot();
}

void* apriltag_manager(void* ptr) {
    lcm_t* lcm = lcm_create(NULL);
    if (!lcm) {
        return NULL;
    }

    apriltag_msmt.initialized = 1;

    printf("Initialized apriltag.\n");

    tag_pose_t_subscribe(lcm, "DETECTIONS", &on_tag_pose, NULL);

    while(rc_get_state()!=EXITING){
        lcm_handle(lcm);
        rc_usleep(100);
    }

    lcm_destroy(lcm);
    return NULL;
}

int apriltag_manager_init() {
    apriltag_msmt.initialized = 0;
    // start thread
    if(rc_pthread_create(&apriltag_manager_thread, &apriltag_manager, NULL,
                SCHED_FIFO, APRILTAGS_MANAGER_PRI)==-1){
        fprintf(stderr, "ERROR in apriltag_manager_init, failed to start thread\n");
        return -1;
    }

    // wait for thread to start
    for(int i=0;i<50;i++){
        if(apriltag_msmt.initialized) return 0;
        rc_usleep(50000);
    }
    fprintf(stderr, "ERROR in apriltag_manager_init, timeout waiting for thread to start\n");
    return -1;
}

int apriltag_manager_cleanup() {
    if(apriltag_msmt.initialized==0){
        fprintf(stderr, "WARNING in apriltag_manager_cleanup, was never initialized\n");
        return -1;
    }
    // wait for the thread to exit
    if(rc_pthread_timed_join(apriltag_manager_thread, NULL, APRILTAGS_MANAGER_TOUT)==1){
        fprintf(stderr,"WARNING: in apriltag_manager_cleanup, thread join timeout\n");
        return -1;
    }
    return 0;
}
