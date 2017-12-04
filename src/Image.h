#ifndef IMAGE_H
#define IMAGE_H
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <vector>
#include <complex>

using namespace std;
using namespace cv;


class ImagePatch
{
  public:

    Mat imag;
    int contourCenterX;
    int contourCenterY;
    vector<vector<Point> > contours;
    vector<Point> MainContour;
    vector<Point> prev_MC;
    double largest_area;
    int middleX,middleY,prevX,prevY;
    bool valid;

    ImagePatch(){
    	valid = false;
    }
   	Point getContourCenter(vector<Point> contour)
    {
    	Moments M = moments(contour,false);
    	Point ret = Point(-1,-1);
    	if(!M.m00)
      {
    		return ret;
      }
    	ret.x = M.m10/M.m00;
    	ret.y = M.m01/M.m00;
    	return ret;
   	}
   	
   	float getContourExtent(vector<Point> contour, Mat patch)
    {
   		
      double area = contourArea(contour);
     	Rect boundRect = boundingRect(contour);
     	float rect_area = boundRect.width * boundRect.height;
      
      if(rect_area > 0)
      {
     		return float(area)/(patch.size[0] * patch.size[1]);
      }
   	}

   	bool Approx(double a, double b, double error){
   		complex<double> mycomplex (a,b);
   		double dist = norm(mycomplex);
   		return (dist < error);
   	}

   	void correctMainContour(int prevX, int prevY){
   		for (int i = 0; i < contours.size(); i++)
   		{	
   			Point a = getContourCenter(contours[i]);
   			if(a != Point(-1, -1))
        {
   				int tmp_x = a.x;
   				int tmp_y = a.y;
   				if(Approx(tmp_x-prevX, tmp_y-prevY, 5))
          {
   					MainContour = contours[i];
   					Point cen = getContourCenter(MainContour);
   					if(cen != Point(-1, -1))
            {
   						contourCenterX = cen.x;
   						contourCenterY = cen.y;
   					}
   				}
   			}
   		}
   	}

    void Process()
    {
    	Mat thresh;
    	cvtColor(imag, thresh, CV_BGR2GRAY);
    	findContours(thresh, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    	prev_MC = MainContour;

      largest_area = 0;
    	int largest_contour_index = -1;
      if (contours.size())
      {

      	for(int i = 0; i < contours.size(); i++) // iterate through each contour. 
        {
       		double a = contourArea(contours[i], false);  //  Find the area of contour
       		
          if( a >= largest_area)
          {
         		largest_area = a;
         		largest_contour_index = i;                //Store the index of largest contour
          }
        }
        	
        MainContour = contours[largest_contour_index];
      	middleX = imag.cols/2;
      	middleY = imag.rows/2;
      	prevX = contourCenterX;
      	prevY = contourCenterY;

      	Point cen = getContourCenter(MainContour);
      	
        if(cen != Point(-1, -1) and getContourExtent(MainContour, imag) >= 0.1)
        {
      		contourCenterX = cen.x;
      		contourCenterY = cen.y;
      		
          if(!Approx(prevX - contourCenterX, prevY - contourCenterY, 5))
          {
      			correctMainContour(prevX, prevY);
      	  }
          //drawContours(imag,contours,largest_contour_index,Scalar(0,255,0),1);
          circle(imag, Point(contourCenterX, contourCenterY), 7, Scalar(255,255,255), -1);
          valid = true;
          circle(imag, Point(middleX, middleY), 2, Scalar(0,0,255), -1);
        }
        else
        {
          contourCenterX = 0;
          contourCenterY = 0;
        }

      }
    }
};
#endif
