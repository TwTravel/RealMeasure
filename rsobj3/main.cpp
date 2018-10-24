#include <opencv2/opencv.hpp>

using namespace cv;

void listcamera()
{
	CvCapture *cap;
    int device_counts = 0;
    while ( true ) {
    cap = cvCreateCameraCapture( device_counts++ );
    if ( cap == NULL ) {
        break;
    }
    cvReleaseCapture(&cap);
}
cvReleaseCapture(&cap);
std::cout << "devices count : " << device_counts - 1 << std::endl;

}

int main(int, char**)
{
	listcamera();
	
   	return 1;
        VideoCapture cap(0); // open the default camera
	VideoCapture cap1(2);
	VideoCapture cap2(3);
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat edges;
    namedWindow("edges",1);
    for(;;)
    {
        Mat frame,frame1,frame2;
        cap >> frame; // get a new frame from camera
		cap1 >> frame1; 
		cap2 >> frame2;
        cvtColor(frame, edges, COLOR_BGR2GRAY);
        //GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        //Canny(edges, edges, 0, 30, 3);
        imshow("edges", frame);
	imshow("frame1", frame1);
	imshow("frame2", frame2);
        if(waitKey(30) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
