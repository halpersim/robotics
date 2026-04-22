#ifndef _LIDAR_THREAD_H_
#define _LIDAR_THREAD_H_

#include <fstream>
#include <eigen/Eigen/Dense>

int send_to_LIDAR(const char data);
int start_LIDAR();
int stop_LIDAR();
int create_LIDAR_socket(int* sock);
int accept_LIDAR_socket(int sock, int* connection);
int read_LIDAR(int sock, int* quality, int* angle, int* distance);

void apply_cox(const Eigen::VectorXd& dist,
               const Eigen::VectorXd& qual,
               Eigen::Vector3d& robot_pose,
               std::ofstream* out) ;

void* lidar_thread(void* arg);

#endif