#ifndef SENSOR_CALC_H
#define SENSOR_CALC_H

typedef struct sensor_calc_msmt {
  float vel[3];
  float rpm[4];
  float rho;
  float T_ref;  // total thrust by 4 rotors to achieve stability
  float normalized_hover_thrust; // normalized hover thrust
  float Tmax;   // max thrust of a rotor
  float phi_ref;
  float theta_ref;
  float throttle[4];
  int fd;
  int initialized;
} sensor_calc_msmt_t;

extern sensor_calc_msmt_t sensor_calc_msmt;

int sensor_calc_manager_init();
int sensor_calc_manager_cleanup();

#endif
