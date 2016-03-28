//OPENCV
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include<opencv2/opencv.hpp>

//ROS
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

//TF
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//C++
#include <sstream> // for converting the command line parameter to integer
#include <iostream>
#include<vector>
#include "../include/clopema_model_fitting/ClopemaGarmentFit.h"

//STD
#include <std_msgs/String.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <stdlib.h>

//#include "opencv2/core/core.hpp"
//#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/nonfree/features2d.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include <opencv2/nonfree/nonfree.hpp>
//#include <opencv2/nonfree/features2d.hpp>


//using namespace Eigen;
using namespace std;
using namespace cv;

//feature extrqction window
int x_resize = 270;
int y_resize = 325;
int width = 860;
int height = 520;

//2D pose values
//feature
Vector2i pt_left_grasp;
Vector2i pt_right_grasp;
//axis
int i_left_grasp = 0;
int i_right_grasp = 0;
Vector2i axis_left;
Vector2i axis_right;

//feature
Vector2i pt_left_drop;
Vector2i pt_right_drop;
//axis
int i_left_drop;
int i_right_drop;


//3D pose parameters
cv_bridge::CvImagePtr depth_ptr;
float f_x = 0.0;
float f_y = 0.0;
float u_0 = 0.0;
float v_0 = 0.0;

//3D publish
tf::Quaternion q_l;
tf::Quaternion q_r;

//flags
int grasp_found = 0;
int drop_found = 0;
int state = 0;

//pub
ros::Publisher pub_left;
ros::Publisher pub_right;
ros::Publisher pub_left_a;
ros::Publisher pub_right_a;

geometry_msgs::Pose pose_msg;
geometry_msgs::Point point_msg;

void getIntrinsic(sensor_msgs::CameraInfo camera_msgs)
{
	f_x = camera_msgs.K[0];
	f_y = camera_msgs.K[4];
	u_0 = camera_msgs.K[2];
	v_0 = camera_msgs.K[5];

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
   	//imshow("Subscriber_Recieved_Data", cv_bridge::toCvShare(msg, "bgr8")->image);
    	Mat image1 = cv_bridge::toCvCopy(msg, "bgr8")->image;
    	ROS_INFO("Data Received from topic image");
    	//imwrite( "home/tahir/Pictures/back1.jpg", image1 );
    	//imwrite("background_stripes.jpg" ,image1);
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

	back1=imread("background_stripes.jpg");


	frame1=image1;
 	//cout<<frame1.rows<<endl;
 	//cout<<frame1.cols<<endl;
  
	//Mat smallFrame1 = frame1(Rect(125,145,400,260));
	//Mat smallBack1 = back1(Rect(125,145,400,260));
	Mat smallFrame1 = frame1(Rect(x_resize,y_resize,width,height));
	Mat smallBack1 = back1(Rect(x_resize,y_resize,width,height));
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
	//imshow ("fore",fore);
	//waitKey(30);

  
    	///2.////////////////////////////////////////////// Contours    
     	medianBlur(fore,fore,9); 
    	imshow ("blured_fore",fore);

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


	//imshow ("testt",out);
     	//waitKey(30);

 
	findContours(out, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE,  Point(0, 0));
	
	
	int k = -1;
        long  max = 0;

   	Mat drawing = Mat::zeros(out.size(), CV_8UC3);

	for(int i = 0; i < contours.size(); i++) 
	{
	   	if(contours.at(i).size() > 80)// && contours.at(i).size()>max ) 
	   	// second threshold will make sure that maximum contout is got 
	   	{
	      		max = contours.at(i).size();
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
	   	approxPolyDP(contours.at(k), approCont,85, true);
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
		Scalar color0(255,0, 0);	//blue
		Scalar color1(0, 255, 0);	//green
		Scalar color2(0,0, 255);	//red
		Scalar color3(255,255,255);	//white
		Scalar color4(0,255, 255);	//yellow
		Scalar color5(255,0,255);	//purple

		for(int i = 0; i < fullApproCont.size(); i++)
		{
			circle(img, fullApproCont.at(i), 8, color2,3,3);
			 //cout<<i<<"="<< fullApproCon.at(i)<<endl;
		}
	
	
		///////////////////////////////// good approximation
		for(int i = 0; i < approCont.size(); i++)
		{
			circle(img, approCont.at(i),15, color1,5,5);
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
	    	for(int i = 0; i < full_contour_points.size(); i++) 
		{
			Vector2i vector1;
			vector1(0) = fullApproCont.at(i).x;
			vector1(1) = fullApproCont.at(i).y;
			full_contour_points.at(i) = vector1;

	    	}

		/////////////////// adding points which croped
	    	contour_points.resize(approCont.size());

		// we store the points in a matrix qnd the other point to produce the vector of approach
		//   | x_point | y_point | x_weight | y_weight |
		MatrixXd image_points(contour_points.size(), 4 );

	    	for(int i = 0; i < contour_points.size(); i++) 
		{
			Vector2i vector2;
			vector2(0) = approCont.at(i).x+x_resize;
			vector2(1) = approCont.at(i).y+y_resize;
			
			//store in matrix
			image_points(i,0) = approCont.at(i).x+x_resize;
			image_points(i,1) = approCont.at(i).y+y_resize;

			contour_points.at(i) = vector2;
			//cout<<"image point"<<i+1<<"="<< vector2(0)<<","<<vector2(1)<<endl;
	
			//draw circles on the detected features
		 	//circle(image1, Point(vector2(0),vector2(1)),15, color1,5,5);
			circle(image1, Point(image_points(i,0),image_points(i,1)),5, color1,5,5);

	   	}
		
		//we select the points we are reaching
		//TODO model fitting to determine the pattern to fold
		
		if (state > 0)
		{	
			//ony check if state 2 == recording features poses
			//cant find features with the arm in the view
			if (state == 2)
			{
				if (grasp_found == 0)
				{
					//for now first take the closes ones to the robot and fold to the longuer ones		
					Vector2i pt_min;
					pt_min.setZero();
					Vector2i pt_min2;
					pt_min2.setZero();
					int min = 0;
					int min2 = 0;

					//grasp goal
					for(int i = 0; i < contour_points.size(); i++) 
					{
						if (pt_min(1) < image_points(i,1))
						{	
							min2 = min;
							pt_min2(1) = pt_min(1);

							min = i;
							pt_min(1) = image_points(i,1);
						}
						else if (pt_min2(1) < image_points(i,1))
						{	
							min2 = i;
							pt_min2(1) = image_points(i,1);
						}
					}
					pt_min(0) = image_points(min,0);
					pt_min2(0) = image_points(min2,0);
					if (image_points(min,0) < image_points(min2,0))
					{
						pt_left_grasp = pt_min;
						pt_right_grasp = pt_min2;
						i_left_grasp = min;
						i_right_grasp = min2;
					}
					else
					{
						pt_right_grasp = pt_min;
						pt_left_grasp = pt_min2;
						i_right_grasp = min;
						i_left_grasp = min2;
					}



					grasp_found = 1;
				}

				//visual check
				//left bottom is yellow
				circle(image1, Point(pt_left_grasp(0),pt_left_grasp(1)),5, color4,5,5);
				//right bottom is purple
				circle(image1, Point(pt_right_grasp(0),pt_right_grasp(1)),5, color5,5,5);

				if (drop_found == 0)
				{		
					//release goal

					Vector2i pt_max;
					pt_max(0)=2000;//more than windows size
					pt_max(1)=2000;//more than windows size
					Vector2i pt_max2;
					pt_max2(0)=2000;//more than windows size
					pt_max2(1)=2000;//more than windows size
					int max = 0;//more than windows size
					int max2 = 0;//more than windows size


					for(int i = 0; i < contour_points.size(); i++) 
					{
						if (pt_max(1) > image_points(i,1))
						{	
							max2 = max;
							pt_max2(1) = pt_max(1);

							max = i;
							pt_max(1) = image_points(i,1);
						}
						else if (pt_max2(1) > image_points(i,1))
						{	
							max2 = i;
							pt_max2(1) = image_points(i,1);
						}
					}

					pt_max(0) = image_points(max,0);
					pt_max2(0) = image_points(max2,0);

					if (image_points(max,0) < image_points(max2,0))
					{
						pt_left_drop = pt_max;
						pt_right_drop = pt_max2;
						i_left_drop = max;
						i_right_drop = max2;
					}
					else
					{
						pt_right_drop = pt_max;
						pt_left_drop = pt_max2;
						i_right_drop = max;
						i_left_drop = max2;
					}

					drop_found = 1;
				}


				//visual check
				//left bottom is blue
				circle(image1, Point(pt_left_drop(0),pt_left_drop(1)),5, color0,5,5);
				//right bottom is green
				circle(image1, Point(pt_right_drop(0),pt_right_drop(1)),5, color1,5,5);

				//find the weight points
				int square = 50;
				for(int i = 0; i < contour_points.size(); i++) 
				{
	
					Mat weight = test1(Rect(Point(image_points(i,0)-square/2-x_resize,image_points(i,1)-square/2-y_resize),Point(image_points(i,0)+square/2-x_resize,image_points(i,1)+square/2-y_resize)));

					Moments mom = moments(weight);
					int x = mom.m10/mom.m00;			
					int y = mom.m01/mom.m00;
			
					image_points(i,2) = image_points(i,0) -square/2 + x;
					image_points(i,3) = image_points(i,1) -square/2 + y;

					circle(image1, Point(image_points(i,2),image_points(i,3)),5, color2,5,5);

			   	}
			}

			//difference between grasp/drop state
			Vector2i pt_left;
			Vector2i pt_right;

			if (state == 2)
			{
			pt_left = pt_left_grasp;
			pt_right = pt_right_grasp;
			axis_left(0) =image_points(i_left_grasp,2);
			axis_left(1) =image_points(i_left_grasp,3);
			axis_right(0) =image_points(i_right_grasp,2);			
			axis_right(1) =image_points(i_right_grasp,3);


			}
			else if (state == 3)
			{
			pt_left = pt_left_drop;
			pt_right = pt_right_drop;
			axis_left(0) =image_points(i_left_drop,2);
			axis_left(1) =image_points(i_left_drop,3);
			axis_right(0) =image_points(i_right_drop,2);			
			axis_right(1) =image_points(i_right_drop,3);

			}
			
	
			//compute 3D position

			Vector3f pt_3D_left;
			Vector3f pt_3D_right;
			Vector3f axis_3D_left;
			Vector3f axis_3D_right;
			//ratio to interpolate 1280x1024 <-> 640x480
			float u_ratio = 0.5;
			float v_ratio = 0.46875;

			//3D feqture
			pt_3D_left(2) = (float)(depth_ptr->image.at<short int>(Point(pt_left(0)*u_ratio,pt_left(1)*v_ratio)))/1000.0;
			pt_3D_left(0) = ((pt_left(0)-u_0)*pt_3D_left(2))/f_x ;
			pt_3D_left(1) = ((pt_left(1)-v_0)*pt_3D_left(2))/f_y ;

			pt_3D_right(2) = (float)(depth_ptr->image.at<short int>(Point(pt_right(0)*u_ratio,pt_right(1)*v_ratio)))/1000.0;
			pt_3D_right(0) = ((pt_right(0)-u_0)*pt_3D_right(2))/f_x ;
			pt_3D_right(1) = ((pt_right(1)-v_0)*pt_3D_right(2))/f_y ;

			//3D axis
			axis_3D_left(2) = (float)(depth_ptr->image.at<short int>(Point(axis_left(0)*u_ratio,axis_left(1)*v_ratio)))/1000.0;
			axis_3D_left(0) = ((axis_left(0)-u_0)*axis_3D_left(2))/f_x ;
			axis_3D_left(1) = ((axis_left(1)-v_0)*axis_3D_left(2))/f_y ;

			axis_3D_right(2) = (float)(depth_ptr->image.at<short int>(Point(axis_right(0)*u_ratio,axis_right(1)*v_ratio)))/1000.0;
			axis_3D_right(0) = ((axis_right(0)-u_0)*axis_3D_right(2))/f_x ;
			axis_3D_right(1) = ((axis_right(1)-v_0)*axis_3D_right(2))/f_y ;

			//publish TF 3D pose for graphic debug
			static tf::TransformBroadcaster br_l;
			tf::Transform transform_l;
			static tf::TransformBroadcaster br_r;
			tf::Transform transform_r;

			static tf::TransformBroadcaster br_l_a;
			tf::Transform transform_l_a;
			static tf::TransformBroadcaster br_r_a;
			tf::Transform transform_r_a;
		
			//publish TF left
			transform_l.setOrigin(tf::Vector3(pt_3D_left(0),pt_3D_left(1),pt_3D_left(2)));
			q_l.setRPY(3.14,0,0);
			transform_l.setRotation(q_l);
			br_l.sendTransform(tf::StampedTransform(transform_l, ros::Time::now(),"/camera_rgb_optical_frame","/left_goal"));

			//publish TF right
			transform_r.setOrigin(tf::Vector3(pt_3D_right(0),pt_3D_right(1),pt_3D_right(2)));
			q_r.setRPY(3.14,0,0);
			transform_r.setRotation(q_r);
			br_r.sendTransform(tf::StampedTransform(transform_r, ros::Time::now(),"/camera_rgb_optical_frame","/right_goal"));

			//publish TF left axis
			transform_l_a.setOrigin(tf::Vector3(axis_3D_left(0),axis_3D_left(1),axis_3D_left(2)));
			q_l.setRPY(3.14,0,0);
			transform_l.setRotation(q_l);
			br_l_a.sendTransform(tf::StampedTransform(transform_l_a, ros::Time::now(),"/camera_rgb_optical_frame","/left_axis"));

			//publish TF right axis
			transform_r_a.setOrigin(tf::Vector3(axis_3D_right(0),axis_3D_right(1),axis_3D_right(2)));
			q_r.setRPY(3.14,0,0);
			transform_r_a.setRotation(q_r);
			br_r_a.sendTransform(tf::StampedTransform(transform_r_a, ros::Time::now(),"/camera_rgb_optical_frame","/right_axis"));

			//make the look up transforms
			tf::StampedTransform s_transform_l;
			tf::StampedTransform s_transform_r;			
			tf::StampedTransform s_transform_l_a;
			tf::StampedTransform s_transform_r_a;
			static tf::TransformListener li_l;
			static tf::TransformListener li_r;
			static tf::TransformListener li_l_a;
			static tf::TransformListener li_r_a;

			try
			{
				li_l.lookupTransform("/world" , "/left_goal" , ros::Time(0),s_transform_l);
			}
			catch(tf::TransformException &ex)
			{
				ROS_ERROR("%s", ex.what());
				ros::Duration(0.5).sleep();
				cout << "failed left goal transform" << endl;
			}

			try
			{
				li_r.lookupTransform("/world" , "/right_goal" , ros::Time(0),s_transform_r);
			}
			catch(tf::TransformException &ex)
			{
				ROS_ERROR("%s", ex.what());
				ros::Duration(0.5).sleep();
				cout << "failed right goal transform" << endl;
			}

			try
			{
				li_l_a.lookupTransform("/world" , "/left_axis" , ros::Time(0),s_transform_l_a);
			}
			catch(tf::TransformException &ex)
			{
				ROS_ERROR("%s", ex.what());
				ros::Duration(0.5).sleep();
				cout << "failed left axis transform" << endl;
			}

			try
			{
				li_r_a.lookupTransform("/world" , "/right_axis" , ros::Time(0),s_transform_r_a);
			}
			catch(tf::TransformException &ex)
			{
				ROS_ERROR("%s", ex.what());
				ros::Duration(0.5).sleep();
				cout << "failed right axis transform" << endl;
			}


			//publish the transformations

			pose_msg.position.x = s_transform_l.getOrigin().x();
			pose_msg.position.y = s_transform_l.getOrigin().y();
			pose_msg.position.z = s_transform_l.getOrigin().z();
			pose_msg.orientation.x = s_transform_l.getRotation().x();
			pose_msg.orientation.y = s_transform_l.getRotation().y();
			pose_msg.orientation.z = s_transform_l.getRotation().z();
			pose_msg.orientation.w = s_transform_l.getRotation().w();
			pub_left.publish(pose_msg);

			pose_msg.position.x = s_transform_r.getOrigin().x();
			pose_msg.position.y = s_transform_r.getOrigin().y();
			pose_msg.position.z = s_transform_r.getOrigin().z();
			pose_msg.orientation.x = s_transform_r.getRotation().x();
			pose_msg.orientation.y = s_transform_r.getRotation().y();
			pose_msg.orientation.z = s_transform_r.getRotation().z();
			pose_msg.orientation.w = s_transform_r.getRotation().w();
			pub_right.publish(pose_msg);

			point_msg.x = s_transform_l_a.getOrigin().x();
			point_msg.y = s_transform_l_a.getOrigin().y();
			point_msg.z = s_transform_l_a.getOrigin().z();
			pub_left_a.publish(point_msg);

			point_msg.x = s_transform_r_a.getOrigin().x();
			point_msg.y = s_transform_r_a.getOrigin().y();
			point_msg.z = s_transform_r_a.getOrigin().z();
			pub_right_a.publish(point_msg);


		}
	//show the workspace	
	rectangle(image1 , Point(x_resize,y_resize), Point(x_resize +width,y_resize+height), color2 , 1,8,0);
		
	}


	resize(image1, image1, Size(image1.cols/1.2, image1.rows/1.2));
	imshow ("final_image",image1);
	imshow ("test1",test1);

	waitKey(10);
	

} 
	

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{


    	depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    	ROS_INFO("Data Received from topic depth");

}

void stateCallback(geometry_msgs::Point state_)
{
	ROS_INFO("State changed");
	state = state_.x;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	//cv::namedWindow("Subscriber_Recieved_Data");


	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub1 = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);

	image_transport::ImageTransport fr(nh);
	image_transport::Subscriber sub2 = fr.subscribe("/camera/depth/image_raw", 1, depthCallback);

	//subscribe to the camera info topic
	ros::Subscriber info = nh.subscribe<sensor_msgs::CameraInfo>("/camera/rgb/camera_info",1,getIntrinsic);
	
	//subsribe the control state
	ros::Subscriber sub3 = nh.subscribe<geometry_msgs::Point>("/state_change",1,stateCallback);
	ros::spin();
	
	//publisher pose
	pub_left = nh.advertise<geometry_msgs::Pose>("/leftgoal",1);
	pub_right = nh.advertise<geometry_msgs::Pose>("/rightgoal",1);

	//publisher axis
	pub_left_a = nh.advertise<geometry_msgs::Point>("/leftaxis",1);
	pub_right_a = nh.advertise<geometry_msgs::Point>("/rightaxis",1);

	destroyWindow("smallFrame1");
	destroyWindow("smallFrame2");
	destroyWindow("countours_simplified1");
	destroyWindow("countours_simplified2");
}











