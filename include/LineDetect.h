#ifndef LINEDETECT_H
#define LINEDETECT_H

#include <ros/ros.h>
#include <vector>
#include "warehouse_interiit/Line.h"
#include "warehouse_interiit/LineArray.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Config.h"

using namespace cv;

void thinningIteration(cv::Mat& img, int iter)
{
    CV_Assert(img.channels() == 1);
    CV_Assert(img.depth() != sizeof(uchar));
    CV_Assert(img.rows > 3 && img.cols > 3);

    cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);

    int nRows = img.rows;
    int nCols = img.cols;

    if (img.isContinuous()) {
        nCols *= nRows;
        nRows = 1;
    }

    int x, y;
    uchar *pAbove;
    uchar *pCurr;
    uchar *pBelow;
    uchar *nw, *no, *ne;    // north (pAbove)
    uchar *we, *me, *ea;
    uchar *sw, *so, *se;    // south (pBelow)

    uchar *pDst;

    // initialize row pointers
    pAbove = NULL;
    pCurr  = img.ptr<uchar>(0);
    pBelow = img.ptr<uchar>(1);

    for (y = 1; y < img.rows-1; ++y) {
        // shift the rows up by one
        pAbove = pCurr;
        pCurr  = pBelow;
        pBelow = img.ptr<uchar>(y+1);

        pDst = marker.ptr<uchar>(y);

        // initialize col pointers
        no = &(pAbove[0]);
        ne = &(pAbove[1]);
        me = &(pCurr[0]);
        ea = &(pCurr[1]);
        so = &(pBelow[0]);
        se = &(pBelow[1]);

        for (x = 1; x < img.cols-1; ++x) {
            // shift col pointers left by one (scan left to right)
            nw = no;
            no = ne;
            ne = &(pAbove[x+1]);
            we = me;
            me = ea;
            ea = &(pCurr[x+1]);
            sw = so;
            so = se;
            se = &(pBelow[x+1]);

            int A  = (*no == 0 && *ne == 1) + (*ne == 0 && *ea == 1) + 
                     (*ea == 0 && *se == 1) + (*se == 0 && *so == 1) + 
                     (*so == 0 && *sw == 1) + (*sw == 0 && *we == 1) +
                     (*we == 0 && *nw == 1) + (*nw == 0 && *no == 1);
            int B  = *no + *ne + *ea + *se + *so + *sw + *we + *nw;
            int m1 = iter == 0 ? (*no * *ea * *so) : (*no * *ea * *we);
            int m2 = iter == 0 ? (*ea * *so * *we) : (*no * *so * *we);

            if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                pDst[x] = 1;
        }
    }

    img &= ~marker;
}

/**
 * Function for thinning the given binary image
 *
 * Parameters:
 * 		src  The source image, binary with range = [0,255]
 * 		dst  The destination image
 */
void thinning(const cv::Mat& src, cv::Mat& dst)
{
    dst = src.clone();
    dst /= 255;         // convert to binary image

    cv::Mat prev = cv::Mat::zeros(dst.size(), CV_8UC1);
    cv::Mat diff;

    do {
        thinningIteration(dst, 0);
        thinningIteration(dst, 1);
        cv::absdiff(dst, prev, diff);
        dst.copyTo(prev);
    } 
    while (cv::countNonZero(diff) > 0);

    dst *= 255;
}


int LDetect(Mat dis, Vec2f v_line, Vec2f h_line){
	Point p = Point(v_line[0],h_line[0]);
	int count_l,count_r;
	count_l = 1;
	count_r = 1;
	for (int i = 0; i < dis.cols; ++i)
	{
		for (int j = -5; j < 5; ++j)
		{
			if(dis.at<uchar>(i,p.y + j)){
				if(i < p.x)
					count_l++;
				else
					count_r++;
			} 
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

bool sortByTheta(const Vec2f &lhs, const Vec2f &rhs) {return lhs[1] < rhs[1];}

bool sortByRho(const Vec2f &lhs, const Vec2f &rhs) {return lhs[0] < rhs[0];}

void PublishLines(ros::NodeHandle nh,Mat &dst,Mat &dis, vector<Vec2f> lines, queue<int> cluster_index)
{

	ros:: Publisher vertical = nh.advertise<warehouse_interiit::Line >(VERTICAL_TOPIC, 10);
	ros:: Publisher other = nh.advertise<warehouse_interiit::LineArray >(HORIZONTAL_TOPIC, 10);
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
		int k = cluster_index.front();          //cluster boundary
		int size = k-j;                         //number of line in cluster
		float r_avg = 0.0, t_avg = 0.0;
		            
		for (; j < k; j++)
		{
		    r_avg = r_avg + lines[j][0] / size ;
		    t_avg = t_avg + lines[j][1] / size ;
		}
		if(t_avg < 40*CV_PI/180)
		{
		   	v_line[0] = r_avg;
		   	v_line[1] = t_avg;
		}
		else
		{
			h_line[0] = r_avg;
		   	h_line[1] = t_avg;
		}
		L = LDetect(dis,v_line,h_line);
		if(t_avg < 40*CV_PI/180)
		{
		  	vertical_line.rho = r_avg;
		   	vertical_line.theta = t_avg;		   	
		   	vertical.publish(vertical_line);
		}
		else
		   	if(t_avg > 40*CV_PI/180 and t_avg<140*CV_PI/180)
		   	{
		   		warehouse_interiit::Line temp;
		   		temp.rho = r_avg;
		   		temp.theta = t_avg;
		   		temp.L = L;
		   		horizontal_lines.lines.push_back(temp);
		   	}
		// if(t_avg > 10*CV_PI/180 and t_avg < 80*CV_PI/180 or t_avg > 100*CV_PI/180)
		//    	continue;
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
		line(dst, pt1, pt2, Scalar(255,0,0), 2, CV_AA);
		cluster_index.pop();
	}
	warehouse_interiit::Line temp;
	temp.rho = 0;
	temp.theta = CV_PI/2;
	std::reverse(horizontal_lines.lines.begin(),horizontal_lines.lines.end());
	horizontal_lines.lines.push_back(temp);
	if(horizontal_lines.lines.size() > 0)
		other.publish(horizontal_lines);
}

void ClusterLines(ros::NodeHandle nh,Mat &dis,Mat &dst)
{
	queue<int> cluster_index;
	float x_i, y_i;
	int zero_index;
	int n_cluster = 1;
	int theta = 0, prev_theta = 0;
	vector<Vec2f> lines;

	cv::cvtColor(dst, dis, CV_BGR2GRAY);
	cv::threshold(dis, dis, 10, 255, CV_THRESH_BINARY);
	thinning(dis, dis);
	cv::imshow("dst", dis);

	HoughLines(dis, lines, 5, CV_PI*3/180,200, 0, 0 );
	for (int i = 0; i < lines.size(); ++i)
	{
		double cos_t = cos(lines[i][1]);  double sin_t = sin(lines[i][1]);
		double x0 = lines[i][0]*cos_t, y0 = lines[i][0]*sin_t;
		double alpha = 3000;

		Point pt1(cvRound(x0 + alpha*(-sin_t)), cvRound(y0 + alpha*cos_t));
		Point pt2(cvRound(x0 - alpha*(-sin_t)), cvRound(y0 - alpha*cos_t));
		line(dst, pt1, pt2, Scalar(255,255,255), 2, CV_AA);
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
	}
	PublishLines(nh,dst,dis,lines,cluster_index);
}

#endif
