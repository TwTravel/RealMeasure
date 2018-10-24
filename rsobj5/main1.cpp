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


 

void Img2OpencvMat(cv::Mat &mat, image&im) {
	  
	  int nr= mat.rows; // number of rows
	  int nc= mat.cols * mat.channels(); // total number of elements per line
	  int w = mat.cols;
	  int h = mat.rows;
	  int i,j,k,c;
	  c =3;
	  
      for (j=0; j<nr; j++) {
		  uchar* data= mat.ptr<uchar>(j);
			  for(i = 0; i < w; ++i){
			    for(k = 0; k < c; ++k){
			    int src_index = i + w*j + w*h*k;
                 *data++= im.data[src_index];
            } // end of row                 
      }
    }

}

void OpencvMat2Img(cv::Mat &mat, image&im) {
	  
	  int nr= mat.rows; // number of rows
	  int nc= mat.cols * mat.channels(); // total number of elements per line
	  int w = mat.cols;
	  int h = mat.rows;
	  int i,j,k,c;
	  c =3;
	  
	   
      for (j=0; j<nr; j++) {
		  uchar* data= mat.ptr<uchar>(j);
			  for(i = 0; i < w; ++i){
			    for(k = 0; k < c; ++k){
			    int src_index = i + w*j + w*h*k;
                im.data[src_index] = *data;
				data ++;
            } // end of row                 
         }
    }

} 
 int main(int, char**)
{
	//#################################################################################
	//#################################################################################
	//#################################################################################
	//#################################################################################
	// make_window("predictions", 512, 512, 0);
	list *options   = read_data_cfg("cfg/coco.data");
    char *name_list = option_find_str(options, "names", "data/names.list");
    char **names    = get_labels(name_list);

    image **alphabet = load_alphabet();
    network *net = load_network("cfg/yolov3.cfg", "../../data/yolov3.weights", 0);
    set_batch_network(net, 1);
    srand(2222222);
    double time;
    char buff[256];
    char *input = buff;
    float nms=.45;
	int filecnt = 0;
	FILE* infilelist =fopen("filelist.txt","rt");
	//#################################################################################
	//#################################################################################
	//#################################################################################
	//#################################################################################
	listcamera();
	
    //	return 1;
    VideoCapture cap(1), cap2(0); // open the default camera
	 
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat edges;
    namedWindow("edges",1);
    for(;;)
    {
        Mat frame,frame1,frame2;
        cap   >> frame; // get a new frame from camera
        cap2  >> frame1;		 
       
		 //#########################################################################
		 //#########################################################################
		 int WIDTH  = frame.cols;
		 int HEIGHT = frame.rows;
         //int cols = mat.cols;
		 float thresh(0.5),  hier_thresh(0.5);
	     image im = make_image(WIDTH, HEIGHT, 3);
		 //getimgdata((char*)rgb_frame_data, im, WIDTH, HEIGHT);
		 OpencvMat2Img(frame, im);
         image sized = letterbox_image(im, net->w, net->h);
        //image sized = resize_image(im, net->w, net->h);
        //image sized2 = resize_max(im, net->w);
        //image sized = crop_image(sized2, -((net->w - sized2.w)/2), -((net->h - sized2.h)/2), net->w, net->h);
        //resize_network(net, sized.w, sized.h);
        layer l = net->layers[net->n-1];


        float *X = sized.data;
        time=what_time_is_it_now();
        network_predict(net, X);
        printf("%s: Predicted in %f seconds.\n", input, what_time_is_it_now()-time);
        int nboxes = 0;
        detection *dets = get_network_boxes(net, im.w, im.h, thresh, hier_thresh, 0, 1, &nboxes);
        printf("detect %d boxes\n", nboxes);
        //if (nms) do_nms_obj(boxes, probs, l.w*l.h*l.n, l.classes, nms);
        if (nms) 
			do_nms_sort(dets, nboxes, l.classes, nms);
        draw_detections(im, dets, nboxes, thresh, names, alphabet, l.classes);
        free_detections(dets, nboxes);
        
		Img2OpencvMat(frame, im);
		/*show_image(im, "predictions", 0);
		char filebuffer[100];
		sprintf(filebuffer,"decout/%05i", filecnt);
		save_image(im, filebuffer);*/
		imshow("edges", frame);
	    imshow("cam2", frame1);	

        filecnt++;
        free_image(im);
        free_image(sized);
			
		//#########################################################################
		//#########################################################################
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
