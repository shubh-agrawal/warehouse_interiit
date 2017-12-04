#include "Image.h"
#include "Utils.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#define N_SLICE_H 9
#define N_SLICE_W 19

using namespace std;
using namespace cv;

Mat img;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "line_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
	// VideoCapture v(argv[1]);
	std::vector<Point> points;
	while(nh.ok())
	{
		Mat dst;	
		if(!img.empty())
		{
			vector<ImagePatch> images(N_SLICE_W*N_SLICE_H);
			imshow("original", img);
			img = RemoveBackground(img, true);
			images = SlicePart(img, images, N_SLICE_H, N_SLICE_W,points);
			dst = RepackImages(images,N_SLICE_H,N_SLICE_W);
			//dst = ShowGrid(dst,N_SLICE_H,N_SLICE_W);
			Mat dis;
			dis = img.clone();
			dis *= 0;
			for (std::vector<Point>::iterator i = points.begin(); i != points.end(); ++i)
			{
				circle(dis, *i, 7, Scalar(255,0,255), -1);
			}
			cvtColor(dis,dis,CV_BGR2GRAY);
			vector<Vec2f> lines;
			HoughLines(dis, lines, 1, CV_PI*30/180, 50, 0, 0 );
			for( size_t i = 0; i < lines.size(); i++ )
  			{	
     			float rho = lines[i][0], theta = lines[i][1];
     			Point pt1, pt2;
     			double a = cos(theta), b = sin(theta);
     			double x0 = a*rho, y0 = b*rho;
     			pt1.x = cvRound(x0 + 2000*(-b));
     			pt1.y = cvRound(y0 + 2000*(a));
     			pt2.x = cvRound(x0 - 2000*(-b));
     			pt2.y = cvRound(y0 - 2000*(a));
     			line( dst, pt1, pt2, Scalar(255,255,255), 3, CV_AA);
  			}
			imshow("Display",dst);
			imshow("Point",dis);
			// imshow("Real",img);
			if(char(waitKey(1)) == 'q')
				break;
		}
		ros::spinOnce();
	}
	return 0;
}