#include <iostream>
#include <fstream>
#include "eigen-5.0.0/Eigen/Dense"
#include "cox_algorithm.h"

const int N = 180;

void apply_cox(const Eigen::VectorXd& dist, const Eigen::VectorXd& qual) {
    Eigen::VectorXd ang(N);

    for(int i=0; i<N; i++) {
        ang[i] = -i * 2.0 / 360.0 * 2.0 * M_PI;
    }

    Eigen::Vector3d robot_pose(500.0, 500.0, 0.0);
	Eigen::Vector3d sensor_pose(0.0, 0.0, 0.0);
	Eigen::MatrixXd linemodel(4,4);

    double x,y,a;
	Eigen::MatrixXd C;

	float ARENA_X = 2425;
	float ARENA_Y = 3630;
    
    linemodel << 0, 0, ARENA_X, 0, 			// South Wall
		     ARENA_X, 0, ARENA_X, ARENA_Y,	// East Wall
		     ARENA_X, ARENA_Y, 0, ARENA_Y,	// North Wall
		     0, ARENA_Y, 0, 0; 			// West Wall

    
    COX_Line_Fit(ang, dist, robot_pose, linemodel, sensor_pose, x, y, a, C);

    std::cout << "[x,y,a]: [" << x << ", " << y << ", " << a << "]" << std::endl;
    std::cout << "Covariance C:\n" << C << std::endl;
}

int main() {

    std::ifstream file_in("testfile_arena.txt");
    std::string s;
    
    int quality, angle, distance;
    int full_rotations = 0;

    int last_angle = -1;

    
    int last_rotation = 1000;
    Eigen::VectorXd dist;
    Eigen::VectorXd qual;

    dist.setZero(N);
    qual.setZero(N);

    while((file_in >> quality >> angle >> distance) && full_rotations < 6){
        if (angle == 0 && last_rotation > 10) {
            if (full_rotations > 0) {
                std::cout << "-----------------------------" << std::endl;

                apply_cox(dist, qual);
            } 
            full_rotations++;
            dist.setZero(N);
            qual.setZero(N);
            last_rotation = 0;
        } else {
            last_rotation++;
        }
        
        if (quality > qual[angle/2]) {
            dist[angle/2] = distance;
            qual[angle/2] = quality;
        }
    }

    return 0;
}