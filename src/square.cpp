#include "Config.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

Mat img;
bool newImage;
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

Mat RemoveBackground(Mat image){
	Mat mask,maskU,maskL, img_erosion, masked_image;
	Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));

	cvtColor(image, image, CV_BGR2HSV);
	inRange(image, Scalar(165,75,75), Scalar(180,255,255), maskU); // Only For segmenting Red
    inRange(image, Scalar(0,75,75), Scalar(10,255,255), maskL);
    addWeighted(maskU, 1.0, maskL, 1.0, 0.0, mask);
    
	bitwise_and(image, image, masked_image, mask);
    erode(masked_image, img_erosion, kernel, Point(-1, -1), 3);
    dilate(img_erosion,img_erosion,kernel,Point(-1,-1),7);
	    
	return img_erosion;
}


int thresh = 50, N = 2;

double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

Point findSquares( const Mat& image, vector<vector<Point> >& squares )
{
    squares.clear();

    Mat pyr, timg, gray0(image.size(), CV_8U), gray;

    Mat out(image);
    dilate(out, out, Mat(), Point(-1,-1));
    medianBlur(out, out, 7);

    pyrDown(out, pyr, Size(out.cols/2, out.rows/2));
    pyrUp(pyr, timg, out.size());
    vector<vector<Point> > contours;

    for( int c = 0; c < 3; c++ )
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);

        for( int l = 0; l < N; l++ )
        {
            if( l == 0 )
            {
                Canny(gray0, gray, 0, thresh, 5);
                dilate(gray, gray, Mat(), Point(-1,-1));
            }
            else
            {
                gray = gray0 >= (l+1)*255/N;
            }

            findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

            vector<Point> approx;

            for( size_t i = 0; i < contours.size(); i++ )
            {

                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

                if( approx.size() == 4 &&
                    fabs(contourArea(Mat(approx))) > 1000 &&
                    isContourConvex(Mat(approx)) )
                {
                    double maxCosine = 0;

                    for( int j = 2; j < 5; j++ )
                    {
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }
                    if( maxCosine < 0.3 )
                        squares.push_back(approx);
                }
            }
        }
    }
}


Point drawSquares( Mat& image, const vector<vector<Point> >& squares )
{
    Point center;
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        polylines(image, &p, &n, 1, true, Scalar(0,255,0), 3, CV_AA);

        for(int j = 0; j < 4; ++j){
            center.x = p[j].x;
            center.y = p[j].y;
        }
        center.x = center.x/4;
        center.y = center.y/4;
        return center;
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "square_detector");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(CAM_TOPIC, 1, imageCallback);
    ros::Publisher centerPub = nh.advertise<geometry_msgs::Point>("square", 5);
    ros::Rate rate(20);
    newImage = false;
    Point center;
    geometry_msgs::Point pt;
    vector<vector<Point> > squares;
    if(!img.empty()){
        img = RemoveBackground(img);
        findSquares(img, squares);
        center = drawSquares(img, squares);
        pt.x = center.x;
        pt.y = center.y;
        centerPub.publish(pt);
        imshow("out", img);
        waitKey(0);
    }

    return 0;
}
