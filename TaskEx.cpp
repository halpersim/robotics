#include <iostream>
#include <unistd.h>
#include "TaskEx.h"

using namespace std;

void* task2(void* args) {
    while (true) {
        cout << "Hello there" << endl;
        sleep(5);
    }

    return NULL;
}