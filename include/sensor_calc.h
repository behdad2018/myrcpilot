#ifndef SENSOR_CALC_H
#define SENSOR_CALC_H

typedef struct sensor_calc_msmt {
  float vel[3];
  float rho;
  int fd;
} sensor_calc_msmt_t;

extern sensor_calc_msmt_t sensor_calc_msmt;

int sensor_calc_manager_init();
int sensor_calc_manager_cleanup();

#endif
