#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "thread_test.h" //camera thread

using namespace cv;
using namespace std;

 

res_xy camera_func(const Mat& frame)
{
	res_xy res;
	res.x = 0; 
	res.y = 0;  
    if (frame.empty()) {
        cerr << "ERROR: Empty frame passed to camera_func" << endl;
        return res;
    }

    Mat hsv, img_mask;
    Mat labels, stats, centroids;

    // Convert BGR -> HSV
    cvtColor(frame, hsv, COLOR_BGR2HSV);

    // Blue color range in HSV
    Scalar lower_c(100, 145, 0);
    Scalar upper_c(130, 211, 255);

    // Create mask
    inRange(hsv, lower_c, upper_c, img_mask);

    // Count blue pixels
    int var = countNonZero(img_mask);

    // Find connected components
    int n_object = connectedComponentsWithStats(img_mask, labels, stats, centroids);

    int a_l = 0;
    int ind = -1;

    for (int i = 1; i < n_object; i++) {
        int a = stats.at<int>(i, CC_STAT_AREA);

        if (a > a_l && a < 100000 && a > 100) {
            a_l = a;
            ind = i;
        }
    }
	double x_ob = 0;
    double y_ob = 0;
    double x_frame = 0;
    double y_frame = 0;
    
    if (ind != -1) { // fix maybe ;) 
        x_ob = centroids.at<double>(ind, 0);
        y_ob = centroids.at<double>(ind, 1);

        x_frame = x_ob - frame.cols / 2.0;
        y_frame = y_ob - frame.rows / 2.0;

        cout << "object " << ind
             << " / x = " << (int)x_frame
             << " / y = " << (int)y_frame
             << " / number of cols = " << frame.cols
             << " / number of rows = " << frame.rows
             << " / area = " << a_l
             << " / blue pixels = " << var
             << endl;
    }

    imshow("Mask", img_mask);
    
	
	res.x = (int)x_frame; 
	res.y = (int)y_frame; 
    return res;
}

void* cameraThread(void* arg)
{
    VideoCapture cap(0);

    if (!cap.isOpened()) {
        cerr << "ERROR: Unable to open the camera" << endl;
        return NULL;
    }

    Mat frame;
    cout << "Start grabbing, press a key on Live window to terminate" << endl;

    int64 lastPrint = 0;
    int x_frame = 0; 
    int y_frame =0; 
    while (true) {
        cap >> frame;

        if (frame.empty()) {
            cerr << "ERROR: Unable to grab from the camera" << endl;
            break;
        }

        int64 now = getTickCount();
        double timeSec = (now - lastPrint) / getTickFrequency();

        if (timeSec > 0.1) {
            res_xy res1 = camera_func(frame);
	    cout 
             << " / x = " << res1.x
             << " / y = " << res1.y
             << endl;
            lastPrint = now;
        }

        imshow("Live", frame);

        int key = waitKey(5);
        key = (key == 255) ? -1 : key;

        if (key >= 0) {
            break;
        }
    }

    cout << "Closing the camera" << endl;
    cap.release();
    destroyAllWindows();
    cout << "bye!" << endl;

    return NULL;
}