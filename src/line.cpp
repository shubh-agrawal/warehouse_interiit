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
#include "LineDetect.h"


using namespace std;
using namespace cv;
using namespace warehouse_interiit;
Mat img;
bool newImage;
warehouse_interiit::Line line_y, line_x;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    newImage = true;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void y_filter_cb(const Line::ConstPtr& msg){
    line_y = *msg;
}

void x_filter_cb(const Line::ConstPtr& msg){
    line_x = *msg;
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "line_publisher");
	ros::NodeHandle nh;
	ros:: Publisher vertical = nh.advertise<warehouse_interiit::Line>(VERTICAL_TOPIC, 10);
	ros:: Publisher other = nh.advertise<warehouse_interiit::LineArray>(HORIZONTAL_TOPIC, 10);
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe(CAM_TOPIC, 1, imageCallback);
	image_transport::Publisher line_pub = it.advertise("/segmented_line", 1);
    ros::Subscriber y_sub = nh.subscribe<Line>("feedback/horizontal", 5, y_filter_cb);
    ros::Subscriber x_sub = nh.subscribe<Line>("feedback/vertical", 5, x_filter_cb);
	ros::Rate rate(15);
	std::vector<Point> points;
	//namedWindow("Display");
	sensor_msgs::ImagePtr line_msg;
	newImage = true;
	while(nh.ok())
	{
		Mat dst,dst_P;
		if(!img.empty() && newImage)
		{
			// vector<ImagePatch> images(N_SLICE_W*N_SLICE_H);
			img = img.t();
			cv::flip(img, img, 0);
			dst = RemoveBackground(img, true);
			// images = SlicePart(img, images, N_SLICE_H, N_SLICE_W,points);
			// dst = RepackImages(images,N_SLICE_H,N_SLICE_W);
			// dst = SlicePartParallel(img,points);
			// imshow("Parallel",dst_P);
			//dst = ShowGrid(dst,N_SLICE_H,N_SLICE_W);
			Mat dis;
			ClusterLines(nh,dis,dst);
			line_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
			line_pub.publish(line_msg);

			double cos_t = cos(line_y.theta);  double sin_t = sin(line_y.theta);
			double x0 = line_y.rho*cos_t, y0 = line_y.rho*sin_t;
			double alpha = 3000;

			Point pty1(cvRound(x0 + alpha*(-sin_t)), cvRound(y0 + alpha*cos_t));
			Point pty2(cvRound(x0 - alpha*(-sin_t)), cvRound(y0 - alpha*cos_t));
			line(dst, pty1, pty2, Scalar(255, 255, 255), 2, CV_AA);

			cos_t = cos(line_x.theta);
			sin_t = sin(line_x.theta);
			x0 = line_x.rho*cos_t, y0 = line_x.rho*sin_t;

			Point ptx1(cvRound(x0 + alpha*(-sin_t)), cvRound(y0 + alpha*cos_t));
			Point ptx2(cvRound(x0 - alpha*(-sin_t)), cvRound(y0 - alpha*cos_t));
			line(dst, ptx1, ptx2, Scalar(255, 255, 255), 2, CV_AA);


			imshow("Display",dst);
			if(char(waitKey(1)) == 'q')
				break;
			newImage = false;
		}
		ros::spinOnce();
	}
	return 0;
}
