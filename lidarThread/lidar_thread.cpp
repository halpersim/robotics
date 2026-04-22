// g++ main.cpp threads.cpp thread_test.cpp lidar_thread.cpp cox_algorithm.cpp -o robot_app \$(pkg-config --cflags --libs opencv) -I/usr/include/eigen3 -lpthread

#include <stdio.h>
#include <iostream>
#include <cstring>
#include <fstream>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <errno.h>
#include <eigen/Eigen/Dense>
#include "cox_algorithm.h"
#include "lidar_thread.h"

using namespace Eigen;

const int N = 180;
const Eigen::Vector3d START_POSE(500.0, 500.0, 0.0);

int send_to_LIDAR(const char data) {
	const char* TCP_IP = "127.0.0.1";
	const int TCP_PORT = 9887;

	int sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0) {
		printf("create Socket failed!\n");
		return 1;
	}

	sockaddr_in server{};
	server.sin_family = AF_INET;
	server.sin_port = htons(TCP_PORT);

	if (inet_pton(AF_INET, TCP_IP, &server.sin_addr) <= 0) {
		printf("Invalid IP Address \n");
		close(sock);
		return 2;
	}


	if (connect(sock, (struct sockaddr*)&server, sizeof(server)) < 0) {
		printf("Connection failed!\n");
		close(sock);
		return 3;
	}

	const char data_packet[] = {data, 0x00};
	
	if (send(sock, data_packet, sizeof(data_packet), 0) < 0) {
		printf("send data failed!\n");
		close(sock);
		return 4;
	}


	close(sock);
	return 0;
}

// starts the lidar by creating a socket and sending the right message
// returns 0, if succeeded of >0 in case of error
int start_LIDAR(){
	return send_to_LIDAR(0x10);
}

// stops the lidar by creating a socket and sending the right message
// returns 0, if succeeded of >0 in case of error
int stop_LIDAR() {
	return send_to_LIDAR(0x20);
}


int create_LIDAR_socket(int* sock) {
	const char* TCP_IP = "127.0.0.1";
	const int TCP_PORT = 9888;

	*sock = socket(AF_INET, SOCK_STREAM, 0);
	if (*sock < 0) {
		printf("create Socket failed!\n");
		return 1;
	}

	sockaddr_in server{};
	server.sin_family = AF_INET;
	server.sin_port = htons(TCP_PORT);
	server.sin_addr.s_addr = inet_addr(TCP_IP);

	if (bind(*sock, (struct sockaddr*)&server, sizeof(server)) < 0) {
		printf("bind failed with code %d\n", errno);
		close(*sock);
		return 2;
	}

	
	if (listen(*sock, 5) < 0) {
		printf("listen failed\n");
		return 3;
	}


	return 0;
}

int accept_LIDAR_socket(int sock, int* connection) {
	sockaddr_in client{};
	socklen_t client_len = sizeof(client);
	*connection = accept(sock, (struct sockaddr*)&client, &client_len);
	if(*connection < 0) {
		printf("accept failed\n");
		return 1;
	}
	
	return 0;
}

int read_LIDAR(int sock, int* quality, int* angle, int* distance) {
	char Header[5];
	char Data[4096];


	int total = 0;

	//receive header
	while(total < 5) {
		int n = recv(sock, Header + total, 5 - total, 0);
		if (n <= 0) {
			return 1;
		}
		total += n;
	}

	if ((unsigned char)Header[0] == 165) {
		int number_of_bytes_in_data = 
			((unsigned char)Header[2] << 16) |
			((unsigned char)Header[3] << 8) |
			((unsigned char)Header[4]);
		
		//receive payload
		int received = 0;
		while(received < number_of_bytes_in_data) {
			int n = recv(sock, Data + received, number_of_bytes_in_data - received, 0);
			if (n <= 0) {
				return 2;
			}
			received += n;
		}

		*quality = ((unsigned char)Data[0])>>2;
		*angle = ((((unsigned char)Data[1]) >> 1) + 
				(((unsigned char)Data[2]) << 8)) >> 7;

		*distance = (((unsigned char)Data[3]) + 
				(((unsigned char)Data[4]) << 8)) >> 2;

		//printf("%d, %d, %d\n", *quality, *angle, *distance);
 	}

	return 0;
}

void apply_cox(const Eigen::VectorXd& dist, const Eigen::VectorXd& qual, Eigen::Vector3d& robot_pose, std::ofstream* out) {
    Eigen::VectorXd ang(N);

    for(int i=0; i<N; i++) {
        ang[i] = -i * 2.0 / 360.0 * 2.0 * M_PI;
    }


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

    robot_pose[0] += x;
    robot_pose[1] += y;
    robot_pose[2] += a;

    //std::cout << "[x,y,a]: [" << x << ", " << y << ", " << a << "]" << std::endl;
    if (out) {
	(*out) << "[x,y,a]: [" << x << ", " << y << ", " << a << "]" << std::endl;
    }
    //std::cout << "Covariance C:\n" << C << std::endl;
}

void* lidar_thread(void* arg) {
	printf("create receiving socket!\n");

	int sock, rec_sock;

	if(create_LIDAR_socket(&sock) > 0) {
		printf("creating receiving socket failed!");
		return NULL;
	}
	
	printf("receiving socket created!\n");

	if (start_LIDAR() > 0) {
        close(sock);
		return NULL;
	}

	printf("LIDAR started!\n");

	if(accept_LIDAR_socket(sock, &rec_sock) > 0) {
		printf("accept failed!\n");
		close(sock);
		stop_LIDAR();
		return NULL;
	}

	printf("start reading messages!\n");

	int quality, angle, distance;
	int full_rotations = 0;
	int last_rotation = 10000; 


	Eigen::Vector3d robot_pose = START_POSE;
	Eigen::Vector3d last_reported_pose = START_POSE;
    Eigen::VectorXd dist;
	Eigen::VectorXd qual;

	dist.setZero(N);
	qual.setZero(N);

	for(int i=0; i<100000; i++) {			
		int rc = read_LIDAR(rec_sock, &quality, &angle, &distance);
        if(rc != 0){
            printf("read_LIDAR failed\n");
            break;
        }

		if (angle == 0 && last_rotation > 10) {
            if (full_rotations > 0) {
				if (full_rotations < 10) {
					std::stringstream ss;

					ss << "debug_data/rot_" << full_rotations << ".txt";
					std::ofstream out(ss.str().c_str());

					out << dist << "\n" << qual;
			        apply_cox(dist, qual, robot_pose, &out);	
				} else {
					apply_cox(dist, qual, robot_pose, NULL);
				}
				
				Eigen::Vector3d diff = robot_pose - last_reported_pose;

				if (diff[0]*diff[0] + diff[1]*diff[1] > 50*50 || abs(diff[2]) > 0.2) {
					Eigen::Vector3d diff_to_start = robot_pose - START_POSE;
					std::cout << "diff to start [x,y,a] = [" << diff_to_start[0] << ", " << diff_to_start[1] << "," << diff_to_start[2] << "]" << std::endl;
					last_reported_pose = robot_pose;
				}
				
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

	printf("Stopping LIDAR...\n");
    stop_LIDAR();
	// if (stop_LIDAR() > 0) {
	// 	return NULL;
	// }

	close(rec_sock);
	close(sock);

    return NULL;
}