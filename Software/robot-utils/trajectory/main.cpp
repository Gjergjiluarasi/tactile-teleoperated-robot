#include <iostream>
#include <math.h>
#include "trajectory.hpp"

using namespace std;

int main()
{
    trajectory_t tr_0 = {
        .x = 10,
        .v = 0,
        .a = 0},
        tr_f = {
        .x = 30,
        .v = 0,
        .a = 0};
    int32_t vMax = 5, aMax = 10;    
    trajectoryParameters_t trajectory_parameters_desired = getTrajectoryParameters(tr_0,tr_f,vMax,aMax);
    cout<<"The desired trajectory parameters are: " <<trajectory_parameters_desired.b0<<" "
                                                    <<trajectory_parameters_desired.b1<<" "
                                                    <<trajectory_parameters_desired.b2<<" "
                                                    <<trajectory_parameters_desired.b3<<" "
                                                    <<trajectory_parameters_desired.b4<<" "
                                                    <<trajectory_parameters_desired.b5<<endl;

    return 0;
}