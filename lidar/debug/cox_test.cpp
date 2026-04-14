#include "eigen-5.0.0/Eigen/Dense"
#include <vector>
#include <cmath>
#include <iostream>
#include <chrono>
#include "cox_algorithm.h"

using namespace Eigen;
using namespace std::chrono;


// --- cart2pol equivalent ---
void cart2pol(const VectorXd& x, const VectorXd& y,
              VectorXd& theta, VectorXd& r)
{
    theta = y.binaryExpr(x, [](double yi, double xi) {
        return std::atan2(yi, xi);
    });

    r = (x.array().square() + y.array().square()).sqrt();
}

int main()
{
    // --- Parameters ---
    double alfa = 0.0;
    double beta = 0.0;
    double gamma = 0.0;

    Vector3d ROBOT_POSE(0.0, 0.0, 0.0);

    // --- Line model ---
    MatrixXd LINEMODEL(2,4);
    LINEMODEL << -4, -5, -4,  5,
                 -5, -5,  5, -5;

    double angle = 30.0 * M_PI / 180.0;
    double pose_x = 0.1;
    double pose_y = -0.05;

    // --- points_world (2 x N) ---
    MatrixXd points_world(2,5);
    points_world << -4, -4,  1, -2, -1,
                     2,  0, -5, -5, -5;

    int N = points_world.cols();

    // --- Homogeneous transform ---
    Matrix3d T;
    T << cos(angle), -sin(angle), pose_x,
         sin(angle),  cos(angle), pose_y,
         0,           0,          1;

    // Convert to homogeneous coordinates
    MatrixXd points_h(3, N);
    points_h.topRows(2) = points_world;
    points_h.row(2).setOnes();

    // Apply transform
    MatrixXd points_rotate = T * points_h;

    // --- cart2pol ---
    VectorXd ANG(N), DIS(N);
    cart2pol(points_rotate.row(0).transpose(),
             points_rotate.row(1).transpose(),
             ANG, DIS);

    // --- Call Cox Line Fit ---
    double ddx, ddy, dda;
    MatrixXd C;

    Vector3d SensorPose(alfa, beta, gamma);

    //for(int i=0; i<100; i++) {
    //    auto start = high_resolution_clock::now();
        COX_Line_Fit(ANG, DIS, ROBOT_POSE, LINEMODEL, SensorPose, ddx, ddy, dda, C);

    //    auto stop = high_resolution_clock::now();
    //    auto duration = duration_cast<microseconds>(stop - start);
    //    std::cout << duration.count() << std::endl;
    //}

    // --- Output ---
    std::cout << "ddx: " << ddx << std::endl;
    std::cout << "ddy: " << ddy << std::endl;
    std::cout << "dda: " << dda << std::endl;
    std::cout << "Covariance C:\n" << C << std::endl;

    return 0;
}