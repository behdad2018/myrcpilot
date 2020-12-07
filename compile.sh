#!/bin/bash
rm *.o
make
rm main.o
gcc main.c -o mythrust -I . ./*.o  -O3 -lm -march=native -mfloat-abi=hard -mfpu=neon -funsafe-math-optimizations -fopenmp-simd -O3 -ffast-math -ftree-vectorize 
