#include "eigen-5.0.0/Eigen/Dense"
#include <vector>
#include <cmath>
#include <iostream>

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
) {
    ddx = 0; ddy = 0; dda = 0;

    const double Rx = POSE(0);
    const double Ry = POSE(1);
    const double Ra = POSE(2);

    const double sALFA = SensorPose(0);
    const double sBETA = SensorPose(1);
    const double sGAMMA = SensorPose(2);

    const int max_iterations = 10;
    const int N = DIS.size();
    const int numLines = LINEMODEL.rows();

    // --- Step 0: Precompute wall normals ---
    MatrixXd wall_unit_vectors(numLines, 2);
    wall_unit_vectors.col(0) = -(LINEMODEL.col(3) - LINEMODEL.col(1));
    wall_unit_vectors.col(1) =  (LINEMODEL.col(2) - LINEMODEL.col(0));
    wall_unit_vectors.rowwise().normalize();

    VectorXd dist_unit_to_wall =
        (wall_unit_vectors.array() *
         LINEMODEL.leftCols(2).array()).rowwise().sum();

    // --- Precompute sensor transform ---
    Matrix3d R_sensor;
    R_sensor << cos(sGAMMA), -sin(sGAMMA), sALFA,
                sin(sGAMMA),  cos(sGAMMA), sBETA,
                0,            0,           1;

    // --- Allocate reusable matrices ---
    MatrixXd Xs(3, N);
    MatrixXd meas_world(3, N);

    MatrixXd dist_meas_to_wall(numLines, N);

    Matrix2d rot90;
    rot90 << 0, -1,
             1,  0;

    for (int iteration = 0; iteration < max_iterations; ++iteration) {

        // --- Step 1: Transform points ---
        Xs.row(0) = (DIS.array() * ANG.array().cos()).transpose();
        Xs.row(1) = (DIS.array() * ANG.array().sin()).transpose();
        Xs.row(2).setOnes();

        Xs = R_sensor * Xs;

        Matrix3d R_world;
        R_world << cos(Ra + dda), -sin(Ra + dda), Rx + ddx,
                   sin(Ra + dda),  cos(Ra + dda), Ry + ddy,
                   0,              0,             1;

        meas_world.noalias() = R_world * Xs;

        // --- Step 2: Vectorized distance computation ---
        // Expand walls and points
        MatrixXd points_rep_int = meas_world.topRows(2);
        MatrixXd points_rep = points_rep_int(placeholders::all,VectorXi::LinSpaced(N*numLines,0,N-1)).transpose();
        
        MatrixXd walls_rep = wall_unit_vectors.replicate(N, 1);          // (N*numLines x 2)

        VectorXd dist_rep = dist_unit_to_wall
                                .replicate(N, 1);             // (N*numLines)

        VectorXd dot_vals =
            (walls_rep.array() * points_rep.array()).rowwise().sum();

        VectorXd dist_all = dist_rep - dot_vals;

        // Reshape → (numLines x N)
        Map<MatrixXd> dist_matrix(dist_all.data(), numLines, N);

        // Find closest walls
        std::vector<int> valid_idx;
        std::vector<int> wall_idx;

        const double threshold = 150.0;

        for (int j = 0; j < N; ++j) {
            Eigen::Index minIdx;
            double minVal = dist_matrix.col(j).cwiseAbs().minCoeff(&minIdx);

            if (minVal < threshold) {
                valid_idx.push_back(j);
                wall_idx.push_back((int)minIdx);
            }
        }

        const int M = valid_idx.size();
        if (M < 3) break;

        // --- Step 3: Build LS system ---
        MatrixXd A(M, 3);
        VectorXd y_vec(M);

        Vector2d pose_xy(Rx + ddx, Ry + ddy);

        for (int k = 0; k < M; ++k) {
            int j = valid_idx[k];
            int w = wall_idx[k];

            Vector2d point = meas_world.block<2,1>(0, j);
            Vector2d diff = point - pose_xy;

            double third =
                wall_unit_vectors.row(w).dot(rot90 * diff);

            A.row(k) << wall_unit_vectors.row(w), third;

            y_vec(k) = dist_unit_to_wall(w)
                     - wall_unit_vectors.row(w).dot(point);
        }

        // --- Solve LS ---
        Vector3d b = A.colPivHouseholderQr().solve(y_vec);

        // --- Covariance ---
        int n = std::max(A.rows(), A.cols());
        double est_var = (y_vec - A * b).squaredNorm() / (n - 4);
        C = (A.transpose() * A).inverse() * est_var;

        // --- Update ---
        ddx += b(0);
        ddy += b(1);
        dda += b(2);

        // --- Convergence ---
        if (std::hypot(b(0), b(1)) < 5 &&
            std::abs(b(2)) < 0.1 * M_PI / 180.0) {
            break;
        }
    }
}

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

    COX_Line_Fit(ANG, DIS, ROBOT_POSE, LINEMODEL,
                 SensorPose, ddx, ddy, dda, C);

    // --- Output ---
    std::cout << "ddx: " << ddx << std::endl;
    std::cout << "ddy: " << ddy << std::endl;
    std::cout << "dda: " << dda << std::endl;
    std::cout << "Covariance C:\n" << C << std::endl;

    return 0;
}