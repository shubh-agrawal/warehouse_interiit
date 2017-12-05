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


#define N_SLICE_H 13
#define N_SLICE_W 15

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

bool sortByTheta(const Vec2f &lhs, const Vec2f &rhs) {return lhs[1] < rhs[1];}
bool sortByRho(const Vec2f &lhs, const Vec2f &rhs) {return lhs[0] < rhs[0];}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "line_publisher");
	ros::NodeHandle nh;
	ros:: Publisher vertical = nh.advertise<warehouse_interiit::Line>("/lines/vertical", 10);
	ros:: Publisher other = nh.advertise<warehouse_interiit::LineArray>("/lines/horizontal", 10);
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/cgo3_camera/image_raw", 1, imageCallback);
	ros::Rate rate(5);
	std::vector<Point> points;
	namedWindow("original");
	namedWindow("Point");
	namedWindow("Display");
	while(nh.ok())
	{
		Mat dst;
		if(!img.empty())
		{
			warehouse_interiit::Line vertical_line;
			warehouse_interiit::LineArray horizontal_lines;
			queue<int> cluster_index;
			float x_i, y_i;
			int zero_index;
			int n_cluster = 1;
			int theta = 0, prev_theta = 0;
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
			HoughLines(dis, lines, 1, CV_PI*30/180, 80, 0, 0 );
			if(lines.size() > 0){
				sort(lines.begin(), lines.end(), sortByTheta);
			 	prev_theta = lines[0][1]*180/CV_PI;
				for( size_t i = 0; i < lines.size(); i++ ){
					float rho = lines[i][0], theta = lines[i][1];
					theta = theta*180/CV_PI;
					if (fabs(theta - prev_theta) > 70)
					{
						n_cluster++;
						cluster_index.push(i);     //store indices where abrupt change was found. They store cluster boundary
						zero_index = i;
						sort(lines.begin() + zero_index, lines.end(),sortByRho);
						float prev_rho = lines[zero_index][0];
						for( size_t r_i = zero_index; r_i < lines.size(); r_i++ )
						{	
							float rho = lines[r_i][0];
						// if(fabs(rho - prev_rho) > 3*img.rows/N_SLICE_H){
							if(fabs(rho - prev_rho) > 3*img.rows/N_SLICE_H){
								cluster_index.push(r_i);
							}
						}
						break;
					}
					prev_theta = theta;
				}
				cluster_index.push(lines.size());

		        // Iterate through each cluster      
		        int j = 0;
		        for( ; !cluster_index.empty(); )
		        {
		            int k = cluster_index.front();          //cluster boundary
		            int size = k-j;                         //number of line in cluster
		            float r_avg = 0.0, t_avg = 0.0;
		            
		            for (; j < k; j++)
		            {
		                r_avg = r_avg + lines[j][0] / size ;
		                t_avg = t_avg + lines[j][1] / size ;
		            }
		            if(t_avg < 10*CV_PI/180){
		            	vertical_line.rho = r_avg;
		            	vertical_line.theta = t_avg;
		            	vertical.publish(vertical_line);
		            }
		            else
		            	if(t_avg > 80*CV_PI/180 and t_avg<100*CV_PI/180){
		            		warehouse_interiit::Line temp;
		            		temp.rho = r_avg;
		            		temp.theta = t_avg;
		            		horizontal_lines.lines.push_back(temp);
		            		other.publish(horizontal_lines);
		            	}
		            if(t_avg > 10*CV_PI/180 and t_avg < 80*CV_PI/180 or t_avg > 100*CV_PI/180)
		            	continue;
		            j = k;                                //update cluster boundary

		            double cos_t = cos(t_avg);  double sin_t = sin(t_avg);
		            double x0 = r_avg*cos_t, y0 = r_avg*sin_t;
		            double alpha = 3000;

					Point pt1(cvRound(x0 + alpha*(-sin_t)), cvRound(y0 + alpha*cos_t));
					Point pt2(cvRound(x0 - alpha*(-sin_t)), cvRound(y0 - alpha*cos_t));

					line(dst, pt1, pt2, Scalar(255,255,255), 2, CV_AA);
					cluster_index.pop();
				}
			}
			imshow("Display",dst);
			imshow("Point",dis);
			// imshow("Real",img);
			if(char(waitKey(1)) == 'q')
				break;
		}
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}