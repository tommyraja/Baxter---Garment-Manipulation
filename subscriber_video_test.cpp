#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <stdio.h>
#include <iostream>
#include "opencv2/opencv.hpp"
//#include "opencv2/core/core.hpp"
//#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/nonfree/features2d.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include <opencv2/nonfree/nonfree.hpp>
//#include <opencv2/nonfree/features2d.hpp>
#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
#include <stdlib.h>
#include "../include/clopema_model_fitting/ClopemaGarmentFit.h"


//using namespace Eigen;
using namespace std;
using namespace cv;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
   	//imshow("Subscriber_Recieved_Data", cv_bridge::toCvShare(msg, "bgr8")->image);
    	Mat image1 = cv_bridge::toCvCopy(msg, "bgr8")->image;
    	ROS_INFO("Data Received from topic");
    	//imwrite( "home/tahir/Pictures/back1.jpg", image1 );
    	//imwrite("Background_10.jpg" ,image1);
	//imshow ("image1",image1);
    	//waitKey(0);
 
	

    	Mat frame1;
	Mat back1;
	Mat fore;

	BackgroundSubtractorMOG bg;
	bg.set("nmixtures", 3); 
	bg.set("backgroundRatio", 0.1); ///// imp parameter
	/////gives more importance to more significant pixels in subtraction
	//// reducing parameter
	//bg.set("fVarMin", 25); 
	//bg.set("detectShadows",0); 

	back1=imread("Background_1.jpg");


	frame1=image1;
 cout<<frame1.rows<<endl;
 cout<<frame1.cols<<endl;
  
	//Mat smallFrame1 = frame1(Rect(125,145,400,260));
	//Mat smallBack1 = back1(Rect(125,145,400,260));
	Mat smallFrame1 = frame1(Rect(300,316,800,490));
	Mat smallBack1 = back1(Rect(300,316,800,490));

	//imshow("frame1",smallFrame1);
	//imshow("frame2",smallBack1);


     
//////////////////////////////////background subtraction

  	     
     Mat frame=smallFrame1;

     //Mat back=smallFrame2;
     
    // string fn("fistFrameFilename");
 
   for (int i =1; i<=2; i++)
    {
        //cap >> frame;

        bg.operator ()(frame,fore,-1);
        //bg.getBackgroundImage(back);
	frame=smallBack1;
	
    }
		imshow ("fore",fore);
	  	//waitKey(30);

  
    ///2.////////////////////////////////////////////// Contours    
     medianBlur(fore,fore,9); 
    //imshow ("blured_fore",fore);

   // waitKey(0);

    	Mat test1 = Mat::zeros(fore.rows,fore.cols, CV_8UC1);




	    for(int i=0; i<fore.rows; i++)
	    {
	
		for(int j=0; j<fore.cols; j++)
		{
			Scalar intensity1 = fore.at<uchar>(i,j);

		  	if (intensity1[0]>100)
		  	{
	   		test1.at<uchar>(i,j)=255;// increased brightness fore

			}
	    	  }

	      }	 
	
//imshow ("test1",test1);




 
    // waitKey(30);


	RNG rng(12345);
	vector<vector<Point> > contours;
	
	vector<vector<Point> > cont;
	vector<Vec4i> hierarchy;
	int thresh, max_thresh;
	thresh =50;
	max_thresh = 255;
    Mat test;
	Mat image;
	Mat out;

	image=test1; ///////////now going to analysing foreground image
	medianBlur(image,image,5);
	
	Canny(image, out, 100,200,3); // out is binary


	 imshow ("testt",out);
     waitKey(30);

 
	findContours(out, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE,  Point(0, 0));
	
	
	int k = 0;
        long  max = 0;

   	Mat drawing = Mat::zeros(out.size(), CV_8UC3);

	for(int i = 0; i < contours.size(); i++) 
	{
	   if(contours.at(i).size() > 100)// && contours.at(i).size()>max ) 
	   // second threshold will make sure that maximum contout is got 
	   {
	      max = contours.at(i).size();
	      cout<<"contour_number ="<<i<<",size ="<<max <<endl;
	      k=i;
	     
	 
	   }
	}
 Scalar color = Scalar(rng.uniform(0, 200), rng.uniform(0, 200), rng.uniform(0, 200));
	 drawContours(drawing, contours, k, color, 5, 0.01, vector<Vec4i>(), 0, Point())  ;
	 //imshow ("drawing",drawing);
	  //waitKey(0);
   	
  

if (k>=0)
	{
	/////// 3. approximation of the contours
		vector<Point> approCont;
		vector<Point> fullApproCont;
	
	
	   	approxPolyDP(contours.at(k), fullApproCont, 3.5, true);
	   	approxPolyDP(contours.at(k), approCont, 80, true);
	   	// threshold in between 40  to 150 ==> safe zone



	// doing something with the starting and ending of the vector
		reverse(approCont.begin(), approCont.end());
		reverse(fullApproCont.begin(), fullApproCont.end());

		/// show approximation

	////initiliztion of some mat's

		Mat img, final_img;
		img = Mat::zeros(image.rows,image.cols, CV_8UC3);
		final_img = Mat::zeros(image.rows,image.cols, CV_8UC3);
		img.setTo(Scalar::all(0));
		final_img.setTo(Scalar::all(0));
	
	
	/// colors to draw
		Scalar color1(0, 255, 0);
		Scalar color2(0,0, 255);

		for(int i = 0; i < fullApproCont.size(); i++)
		{
			circle(img, fullApproCont.at(i), 8, color2,3,3);
			 //cout<<i<<"="<< fullApproCon.at(i)<<endl;
		}
		cout<<endl;
	
	
		///////////////////////////////// good approximation
		for(int i = 0; i < approCont.size(); i++)
		{
			circle(img, approCont.at(i),8, color1,3,3);

			cout<<i+1<<"="<< approCont.at(i)<<endl;
		}
	
	

	//////////////////////// removing extra features  not needed yet)

		final_img=img+drawing;

		Mat draw_image=final_img;
		Mat draw_image1=img;
		Mat draw_image2=drawing;
	
		//circle(img, Point(349,30),20, color1,3,3);
		//circle(img, Point(348,32),20, color2,3,3);


		//resize(draw_image1, draw_image1, Size(draw_image1.cols/2, draw_image1.rows/2));
		//resize(draw_image2, draw_image2, Size(draw_image2.cols/2, draw_image2.rows/2));
		//resize(final_img, final_img, Size(final_img.cols/2, final_img.rows/2));
	
	
		//imshow("countours_simplified0",draw_image2);
		//waitKey(0);
		//imshow("countours_simplified1",draw_image1);
		//waitKey(0); 	
		//imshow("countours_simplified2",final_img);
		//waitKey(0);  
	
	
	////////////////////////////////////////////////////////// NEXt DAy 
	//////////////////////////////////////////////////// Matching Model	
		vector<Vector2i, aligned_allocator<Vector2i> > contour_points, full_contour_points;
	
		/// Conversion between approximate Points and Vector2i
		 full_contour_points.resize(fullApproCont.size());
	    for(int i = 0; i < full_contour_points.size(); i++) {
		Vector2i vector1;
		vector1(0) = fullApproCont.at(i).x;
		vector1(1) = fullApproCont.at(i).y;
		full_contour_points.at(i) = vector1;
	    }
	/////////////////// adding points which croped
	    contour_points.resize(approCont.size());
	    for(int i = 0; i < contour_points.size(); i++) {
		Vector2i vector2;
		vector2(0) = approCont.at(i).x+300;
		vector2(1) = approCont.at(i).y+316;
		contour_points.at(i) = vector2;
		cout<<"image point"<<i+1<<"="<< vector2(0)<<","<<vector2(1)<<endl;
		 circle(image1, Point(vector2(0),vector2(1)),8, color1,3,3);
	    }

		resize(image1, image1, Size(image1.cols/1.2, image1.rows/1.2));
}
		 imshow ("final_image",image1);
		  waitKey(30);
	
//*/
} 
	







int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	//cv::namedWindow("Subscriber_Recieved_Data");
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);
	ros::spin();
	
	destroyWindow("smallFrame1");
	destroyWindow("smallFrame2");
	destroyWindow("countours_simplified1");
	destroyWindow("countours_simplified2");
}











