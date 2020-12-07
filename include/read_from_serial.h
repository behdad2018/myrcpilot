#pragma once

float read_from_serial();

typedef struct data_t{
		float vel[3];	
		float rho[1];
} data_t;

data_t data;

