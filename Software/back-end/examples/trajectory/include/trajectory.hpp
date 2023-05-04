#pragma once
#include <eigen3/Eigen/Dense>
#include <math.h>

typedef struct{
    double b0;
    double b1;
    double b2;
    double b3;
    double b4;
    double b5;
    }trajectoryParameters_t;

typedef struct{
    int32_t x;
    int32_t v;
    int32_t a;
    }trajectory_t;

trajectoryParameters_t  getTrajectoryParameters(trajectory_t tr_0, trajectory_t tr_f, int32_t vMax, int32_t aMax);
