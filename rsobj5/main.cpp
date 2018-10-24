#include <opencv2/opencv.hpp>
#include "darknet.h"
using namespace cv;

void listcamera()
{
	CvCapture *cap;
    int device_counts = 0;
    //while ( true ) 
	for(int i = 0 ; i < 20 ; i++ )
	{
    cap = cvCreateCameraCapture( device_counts++ );
    if ( cap == NULL ) {
        //break;
    }
	else
	{
		 printf("device %i is ok\n", device_counts);
		  cvReleaseCapture(&cap);
	}
  
}
cvReleaseCapture(&cap);
//std::cout << "devices count : " << device_counts - 1 << std::endl;

}


/*Mat image_to_Mat(image img, int w, int h, int depth, int c)
{
   int i, j, k, count= 0; 
   IplImage* src= cvCreateImage(cvSize(w, h), depth, c);
  
    for(k= 0; k < c; ++k){
        for(i = 0; i < h; ++i){
            for(j = 0; j < w; ++j){
		src->imageData[i*step + j*c + k] = img.data[count++];
		}
	     }
          }
   cvCvtColor(src, src, CV_RGB2BGR);
   cv::Mat dst = cv::cvarrToMat(src, true);
   cvReleaseImage(&src);
    
   return dst;
}*/

void colorReduce0(cv::Mat &image, int div=64) {
	  int nr= image.rows; // number of rows
	  int nc= image.cols * image.channels(); // total number of elements per line
      for (int j=0; j<nr; j++) {
		  uchar* data= image.ptr<uchar>(j);
          for (int i=0; i<nc; i++) {
                  data[i]=int(data[i]/div)*div + div/2;
            }                  
      }
}
 
 
 int main(int, char**)
{
	listcamera();
	
	return 1;
    VideoCapture cap(1); // open the default camera
	 
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat edges;
    namedWindow("edges",1);
    for(;;)
    {
        Mat frame,frame1,frame2;
        cap >> frame; // get a new frame from camera
		 
       
		imshow("edges", frame);
		
		 
        if(waitKey(30) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

/*int main(int, char**)
{
	listcamera();
	
	//return 1;
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
        colorReduce0(frame, 2);
		imshow("edges", frame);
		
		imshow("frame1", frame1);
		imshow("frame2", frame2);
        if(waitKey(30) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}*/