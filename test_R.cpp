// File to test the camera and detect blue color using HSV
// compile with:
// g++ $(pkg-config --libs --cflags opencv) -o camera_test main.cpp
// run with:
// ./camera_test

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

int main(int argc,char ** argv)
{
    VideoCapture cap(0);

    if (!cap.isOpened()) {
        cerr << "ERROR: Unable to open the camera" << endl;
        return 0;
    }

    Mat frame, hsv, img_mask;
    Mat labels, stats, centroids; // for centering 
    int64 lastPrint = 0;

    cout << "Start grabbing, press a key on Live window to terminate" << endl;

    while (1) {

        cap >> frame;

        if (frame.empty()) {
            cerr << "ERROR: Unable to grab from the camera" << endl;
            break;
        }
		
		// mask -> 
		
        // Convert BGR -> HSV
        cvtColor(frame, hsv, COLOR_BGR2HSV);

        // color range in HSV
        Scalar lower_c(160, 50, 50);
        Scalar upper_c(179, 255, 255);

        // Create mask
        inRange(hsv, lower_c, upper_c, img_mask);
		
		
        // Count blue pixels
        int var = countNonZero(img_mask); 
		
		//int n_object = connectComponentsWithStats(img_mask, labels,stats,centroids); 
		int n_object = connectedComponentsWithStats(img_mask, labels, stats, centroids);

		
		for (int i = 1; i<n_object;i++){
			int a = stats.at<int>(i,CC_STAT_AREA); 
			if(a < 500){
				continue; }//filter noise 
			double x_ob = centroids.at<double>(i,0);
			double x_frame = x_ob - frame.cols/2; // left: <0, right : >0  
			double y_ob = centroids.at<double>(i,1);
			double y_frame = y_ob - frame.rows/2; 
			
			//norm
			double norm = x_ob/frame.cols; 
			
			
			// print
			int64 now = getTickCount();
			double timeSec = (now - lastPrint) / getTickFrequency();
			if (timeSec > 0.01) {
				cout <<"object"<<i
				<< "/ x = "<<(int)x_frame
				<< "/ y = "<<(int)y_frame
				<<"/ number of cols=" << frame.cols
				<<"/ number of rows=" << frame.cols
				<<"/ area = "<< a<<endl; 
			}
			}
		
		//clock 
        // time control (0.01 seconds)
        int64 now = getTickCount();
        double timeSec = (now - lastPrint) / getTickFrequency();
        if (var > 500 && timeSec > 3) {
            cout << "blue" << endl;
            lastPrint = now;
        }
        
        // masks end <-

        // Show camera and mask
        imshow("Live", frame);
        imshow("C Mask", img_mask);

        int key = waitKey(5);
        key = (key==255) ? -1 : key;

        if (key >= 0)
            break;
    }

    cout << "Closing the camera" << endl;

    cap.release();
    destroyAllWindows();

    cout << "bye!" << endl;

    return 0;
}
