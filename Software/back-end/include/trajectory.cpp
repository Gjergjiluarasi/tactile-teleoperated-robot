#include "trajectory.hpp"

trajectoryParameters_t getTrajectoryParameters(trajectory_t tr_0, trajectory_t tr_f, int32_t vMax, int32_t aMax)
{
    trajectoryParameters_t trajectoryParams;
    Eigen::Matrix3d T;
    Eigen::Vector3d s, b;

    float timeForVMax = abs(vMax - tr_0.v) / aMax;
    float timeForXFinal = abs(tr_f.x - tr_0.x) / vMax;

    float t = (timeForVMax > timeForXFinal) ? 2 * timeForVMax : timeForXFinal;

    T << pow(t, 3), pow(t, 4), pow(t, 5),
        3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4),
        6 * t, 12 * pow(t, 2), 20 * pow(t, 3);

    s << tr_f.x - tr_0.x - tr_0.v * t - tr_0.a * pow(t, 2) / 2,
        tr_f.v - tr_0.v - tr_0.a * t,
        tr_f.a - tr_0.a;

    b = T.inverse() * s;

    trajectoryParams = {
        .b0 = (double)tr_0.x,
        .b1 = (double)tr_0.v,
        .b2 = (double)tr_0.a / 2,
        .b3 = b[0],
        .b4 = b[1],
        .b5 = b[2]};

    return trajectoryParams;
}