#include <pthread.h>
#include <iostream>
#include "threads.h"
#include "thread_test.h" // camera thread
#include "lidar_thread.h" // lidar thread


using namespace std;

void startThreads() {
    pthread_t camThread;
    pthread_t lidrThread;
    

    int rc1 = pthread_create(&camThread, NULL, cameraThread, NULL);
    int rc2 = pthread_create(&lidrThread, NULL, lidar_thread, NULL);

    // int rc2 = pthread_create(&taskThread, NULL, task2, NULL);

    // if (rc1 != 0) {
    //     cerr << "ERROR: Failed to create camera thread" << endl;
    //     return;
    // }

    // int rc3 = 0; // placceholder for odomerty thread

    if(rc1 != 0 || rc2 != 0){
        cerr << "ERROR: Failed to create threads" << endl;
        return;
    }

    pthread_join(camThread, NULL);
    pthread_join(lidrThread, NULL);

}