#ifndef LINEDETECT_H
#define LINEDETECT_H

#include <ros/ros.h>
#include <vector>
#include "warehouse_interiit/Line.h"
#include "warehouse_interiit/LineArray.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Config.h"
#include <std_msgs/Bool.h>

using namespace cv;

int LDetect(std::vector<Point> points, Vec2f v_line, Vec2f h_line,int sectionY,int sectionX){
	Point p = Point(v_line[0],h_line[0]);
	int count_l,count_r;
	count_l = 1;
	count_r = 1;
	for (std::vector<Point>::iterator i = points.begin(); i != points.end(); ++i)
	{
		if(i->y > p.y - sectionY and i->y < p.y + sectionY){
			if(i->x > p.x + sectionX)
				count_r++;
			if(i->x < p.x - sectionX)
				count_l++;
		}
	}
	 // ROS_INFO("%d %d\n",p.x,p.y);
	float ratio = float(count_r)/float(count_l);
	
	if(ratio > POINT_RATIO){
		ROS_INFO("%f Right",ratio);
		return 1;
	}
	if(ratio < 1/POINT_RATIO){
		ROS_INFO("%f Left",ratio);
		return -1;
	}
	return 0;
}

bool scan = false;

void scanCallBack(const std_msgs::Bool::ConstPtr& Reached)
{
  scan = Reached->data;
}

bool sortByTheta(const Vec2f &lhs, const Vec2f &rhs) {return lhs[1] < rhs[1];}

bool sortByRho(const Vec2f &lhs, const Vec2f &rhs) {return lhs[0] < rhs[0];}

bool sortByX(const Point &l, const Point &r ){ return l.x < l.x; }

void PublishVLines(ros::NodeHandle nh,Mat &dst, vector<Vec2f> lines, queue<int> cluster_index,warehouse_interiit::Line V){
	ros:: Publisher vertical = nh.advertise<warehouse_interiit::LineArray >(VERTICAL_TOPIC, 10);
	warehouse_interiit::LineArray vertical_lines;
	ros::Rate rate(15);

	if(V.L == 1){
		vertical_lines.lines.push_back(V);
	}
	int j = 0;
	while(!cluster_index.empty()){

		ros::spinOnce();
		rate.sleep();
		int k = cluster_index.front();          //cluster boundary
		int size = k-j;                         //number of line in cluster
		float r_avg = 0.0, t_avg = 0.0;

		for (; j < k; j++)
		{
		    r_avg = r_avg + lines[j][0] / size ;
		    t_avg = t_avg + lines[j][1] / size ;
		}

		if(t_avg < 10*CV_PI/180)
		{
		  	warehouse_interiit::Line temp;
			temp.rho = r_avg;
			temp.theta = t_avg;
			temp.L = -1;
			vertical_lines.lines.push_back(temp);
		}

		j = k;                                //update cluster boundary

		double cos_t = cos(t_avg);  double sin_t = sin(t_avg);
		double x0 = r_avg*cos_t, y0 = r_avg*sin_t;
		double alpha = 3000;

		Point pt1(cvRound(x0 + alpha*(-sin_t)), cvRound(y0 + alpha*cos_t));
		Point pt2(cvRound(x0 - alpha*(-sin_t)), cvRound(y0 - alpha*cos_t));
		line(dst, pt1, pt2, Scalar(255,255,255), 2, CV_AA);
		cluster_index.pop();
	}
	vertical.publish(vertical_lines);
}

warehouse_interiit::Line PublishHLines(ros::NodeHandle nh,Mat &dst, vector<Vec2f> lines, queue<int> cluster_index,std::vector<Point> points)
{

	//ros:: Publisher vertical = nh.advertise<warehouse_interiit::LineArray >(VERTICAL_TOPIC, 10);
	ros:: Publisher other = nh.advertise<warehouse_interiit::LineArray >(HORIZONTAL_TOPIC, 15);
	ros::Subscriber scan_sub = nh.subscribe("/edge",10, &scanCallBack);	

	warehouse_interiit::Line vertical_line;
	warehouse_interiit::LineArray horizontal_lines;
	ros::Rate rate(15);
    // Iterate through each cluster  
    Vec2f v_line,h_line;   
    int L; 
    int j = 0;
    while(!cluster_index.empty())
    {
    	ros::spinOnce();
		rate.sleep();
		int k = cluster_index.front();          //cluster boundary
		int size = k-j;                         //number of line in cluster
		float r_avg = 0.0, t_avg = 0.0;
		            
		for (; j < k; j++)
		{
		    r_avg = r_avg + lines[j][0] / size ;
		    t_avg = t_avg + lines[j][1] / size ;
		}
		if(t_avg < 10*CV_PI/180)
		{
		   	v_line[0] = r_avg;
		   	v_line[1] = t_avg;
		}
		else
		{
			h_line[0] = r_avg;
		   	h_line[1] = t_avg;
		}
		L = LDetect(points,v_line,h_line,int(dst.rows/N_SLICE_H),int(dst.cols/N_SLICE_W));
		// if(t_avg < 10*CV_PI/180)
		// {
		//   	vertical_line.rho = r_avg;
		//    	vertical_line.theta = t_avg;		   	
		//    	vertical.publish(vertical_line);
		// }
		// else
		if(t_avg > 80*CV_PI/180 and t_avg<100*CV_PI/180)
		{
			warehouse_interiit::Line temp;
			temp.rho = r_avg;
			temp.theta = t_avg;
			temp.L = L;
			horizontal_lines.lines.push_back(temp);
		}
		if(t_avg > 10*CV_PI/180 and t_avg < 80*CV_PI/180 or t_avg > 100*CV_PI/180)
		   	continue;
		// if(L)
		// {
		// 	if(L == -1) ROS_INFO("Left L\t");
		// 	else 
		// 		if(L == 1) ROS_INFO("Right L\t");
		// }

		j = k;                                //update cluster boundary

		double cos_t = cos(t_avg);  double sin_t = sin(t_avg);
		double x0 = r_avg*cos_t, y0 = r_avg*sin_t;
		double alpha = 3000;

		Point pt1(cvRound(x0 + alpha*(-sin_t)), cvRound(y0 + alpha*cos_t));
		Point pt2(cvRound(x0 - alpha*(-sin_t)), cvRound(y0 - alpha*cos_t));
		line(dst, pt1, pt2, Scalar(255,255,255), 2, CV_AA);
		cluster_index.pop();
	}
	if(!scan){
		vertical_line.rho = -999;
		vertical_line.theta = 90;
		vertical_line.L = 0;
	}
	else{
		vertical_line.rho = 0;
		int count = 0;
		sort(points.begin(),points.end(),sortByX);
		for(int row = dst.rows - 2*dst.rows/N_SLICE_H; row < dst.rows + 2*dst.rows/N_SLICE_H; row++)
		for (int col = 1; col < dst.cols; col ++){
			if(dst.at<Vec3b>(col,row)[0] == 0 and 
				dst.at<Vec3b>(col,row)[1] == 0 and
				dst.at<Vec3b>(col,row)[2] == 0 and
				dst.at<Vec3b>(col - 1,row)[0] != 0 and
				dst.at<Vec3b>(col - 1,row)[1] != 0 and
				dst.at<Vec3b>(col - 1,row)[2] != 0 ){
				vertical_line.rho += col;
				count++;
			}
		}
		vertical_line.rho /= count;
		vertical_line.theta = v_line[1];
		vertical_line.L = 1;
	}
	warehouse_interiit::Line temp;
	temp.rho = 0;
	temp.theta = CV_PI/2;
	std::reverse(horizontal_lines.lines.begin(),horizontal_lines.lines.end());
	horizontal_lines.lines.push_back(temp);
	other.publish(horizontal_lines);
	return vertical_line;
}

void ClusterLines(ros::NodeHandle nh,Mat &dis,Mat &dst,std::vector<Point> points)
{
	queue<int> cluster_index;
	float x_i, y_i;
	int zero_index;
	int n_cluster = 1;
	int theta = 0, prev_theta = 0;
	vector<Vec2f> lines;
	warehouse_interiit::Line v;
	HoughLines(dis, lines, 1, CV_PI*30/180, 100, 0, 0 );
	if(lines.size() > 0)
	{
		sort(lines.begin(), lines.end(), sortByTheta);
	 	prev_theta = lines[0][1]*180/CV_PI;
		for( size_t i = 0; i < lines.size(); i++ )
		{
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
					if(fabs(rho - prev_rho) > 3*dis.rows/N_SLICE_H){
						cluster_index.push(r_i);
					}
					prev_rho = rho;
				}
				break;
			}
			prev_theta = theta;
		}
		cluster_index.push(lines.size());
		v = PublishHLines(nh,dst,lines,cluster_index,points);
	}
	n_cluster = 1;
	theta = 0, prev_theta = 0;

	while(!cluster_index.empty()){
		cluster_index.pop();
	}

	if(lines.size() > 0)
	{
		sort(lines.begin(), lines.end(), sortByTheta);
	 	prev_theta = lines[0][1]*180/CV_PI;
		for( size_t i = 0; i < lines.size(); i++ )
		{
			float rho = lines[i][0], theta = lines[i][1];
			theta = theta*180/CV_PI;
			if (fabs(theta - prev_theta) > 70)
			{
				n_cluster++;     //store indices where abrupt change was found. They store cluster boundary
				zero_index = i;
				sort(lines.begin(), lines.begin() + zero_index,sortByRho);
				float prev_rho = lines[zero_index][0];
				for( size_t r_i = 0; r_i < zero_index; r_i++ )
				{	
					float rho = lines[r_i][0];
				// if(fabs(rho - prev_rho) > 3*img.rows/N_SLICE_H){
					if(fabs(rho - prev_rho) > 3*dis.rows/N_SLICE_W){
						cluster_index.push(r_i);
					}
					prev_rho = rho;
				}
				break;
			}
			prev_theta = theta;
		}

		PublishVLines(nh,dst,lines,cluster_index,v);
	}
}
#endif

