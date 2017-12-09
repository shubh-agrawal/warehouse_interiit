#include "Image.h"
#include "Utils.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "warehouse_interiit/Line.h"
#include "warehouse_interiit/LineArray.h"
#include "LineIn.h"

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
	ros::init(argc, argv, "line_in_publisher");
	ros::NodeHandle nh;
	ros:: Publisher vertical = nh.advertise<warehouse_interiit::LineArray>(VERTICAL_TOPIC, 10);
	ros:: Publisher other = nh.advertise<warehouse_interiit::LineArray>(HORIZONTAL_TOPIC, 10);
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe(CAM_TOPIC, 1, imageCallback);
	ros::Rate rate(15);
	std::vector<Point> points;
	//namedWindow("original");
	//namedWindow("Point");
	namedWindow("Display");
	while(nh.ok())
	{
		Mat dst;
		if(!img.empty())
		{
			vector<ImagePatch> images(N_SLICE_W*N_SLICE_H);
	//		imshow("original", img);
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
			ClusterLines(nh,dis,dst,points);
			imshow("Display",dst);
	//		imshow("Point",dis);
			if(char(waitKey(1)) == 'q')
				break;
		}
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}