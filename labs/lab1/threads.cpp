#include <pthread.h>
#include <iostream>
#include "threads.h"
#include "thread_test.h"
#include "TaskEx.h"

using namespace std;

void startThreads() {
    pthread_t camThread;
    pthread_t taskThread;

    int rc1 = pthread_create(&camThread, NULL, cameraThread, NULL);
    if (rc1 != 0) {
        cerr << "ERROR: Failed to create camera thread" << endl;
        return;
    }

    int rc2 = pthread_create(&taskThread, NULL, task2, NULL);
    if (rc2 != 0) {
        cerr << "ERROR: Failed to create task thread" << endl;
        return;
    }

    pthread_join(camThread, NULL);
    pthread_join(taskThread, NULL);
}