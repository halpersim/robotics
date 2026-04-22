#pragma once

#include <eigen/Eigen/Dense>

using namespace Eigen;

/*
    behaves like the matlab algorithm from the intelligent vehicles course


    ANG         (Nx1) matrix with the measured angles
    DIS         (Nx1) matrix with the measured distances
    POSE        (3x1) vector initial guess of the pose
    LINEMODEL   (Mx4) model of the surrounding, where each line is defined as (start_x, start_y, end_x, end_y)
    SensorPose  (3x1) vector describing the position offset of the the sensor and the robot
    ddx         double scalar describing the offset of the x-coordinate of POSE and the actual pose (as determined by the cox algorithm)
    ddy         double scalar describing the offset of the y-coordinate of POSE and the actual pose (as determined by the cox algorithm)
    dda         double scalar describing the offset in the rotation component of POSE and the actual pose (as determined by the cox algorithm)
    C           (3x3) matrix containg the uncertainty of the cox algorithm estimate
*/
void COX_Line_Fit(
    const VectorXd& ANG,
    const VectorXd& DIS,
    const Vector3d& POSE,
    const MatrixXd& LINEMODEL,
    const Vector3d& SensorPose,
    double& ddx, double& ddy, double& dda,
    MatrixXd& C
);