#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "Config.h"
#include "warehouse_interiit/Line.h"

using namespace std;
using namespace cv;

Mat image;

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

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    image = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
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

void findSquares( const Mat& image, vector<vector<Point> >& squares )
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


int main(int argc, char** argv)
{
    vector<vector<Point> > squares;
    ros::init(argc, argv, "square_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(CAM_TOPIC, 1, imageCallback);
    ros:: Publisher vertical = nh.advertise<warehouse_interiit::Line>("/square/vertical", 10);
    ros:: Publisher horizontal = nh.advertise<warehouse_interiit::Line>("/square/horizontal", 10);

    ros::Rate rate(15);
    while(nh.ok())
    {
        if(!image.empty())
        {
            image = RemoveBackground(image);
            findSquares(image, squares);
            // imshow("out", image);
            waitKey(30);
            float x = 0,y = 0;
            if(squares.size() != 0){
                for (std::vector<std::vector<Point> >::iterator i = squares.begin(); i != squares.end(); ++i)
                {
                    for (std::vector<Point>::iterator j = i->begin(); j != i->end(); ++i)
                    {
                        x += j->x;
                        y += j->y;
                    }
                }
                x /= 4*squares.size();
                warehouse_interiit::Line v , h;
                v.rho = x;
                v.theta = 0;
                h.rho = y;
                h.theta = CV_PI/2;
                vertical.publish(v);
                horizontal.publish(h);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
