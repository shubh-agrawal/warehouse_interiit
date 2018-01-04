#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "zbar.h"
#include <iostream>
#include <iomanip>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include "Config.h"
#include <std_msgs/Bool.h>
// #include "barcode.h"

using namespace std;
using namespace cv;
using namespace zbar;

cv::Mat frame;
cv_bridge::CvImage img_bridge;
sensor_msgs::Image img_msg;
bool scan = false;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    frame = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
void scanCallBack(const std_msgs::Bool::ConstPtr& Reached)
{
  scan = Reached->data;
}

void sharpen(const Mat& img, Mat& result)
{    
    result.create(img.size(), img.type());
    //Processing the inner edge of the pixel point, the image of the outer edge of the pixel should be additional processing
    for (int row = 1; row < img.rows-1; row++)
    {
        //Front row pixel
        const uchar* previous = img.ptr<const uchar>(row-1);
        //Current line to be processed
        const uchar* current = img.ptr<const uchar>(row);
        //new row
        const uchar* next = img.ptr<const uchar>(row+1);
        uchar *output = result.ptr<uchar>(row);
        int ch = img.channels();
        int starts = ch;
        int ends = (img.cols - 1) * ch;
        for (int col = starts; col < ends; col++)
        {
            //The traversing pointer of the output image is synchronized with the current row, and each channel value of each pixel in each row is given a increment, because the channel number of the image is to be taken into account.
            *output++ = saturate_cast<uchar>(5 * current[col] - current[col-ch] - current[col+ch] - previous[col] - next[col]);
        }
    } //end loop
    //Processing boundary, the peripheral pixel is set to 0
    result.row(0).setTo(Scalar::all(0));
    result.row(result.rows-1).setTo(Scalar::all(0));
    result.col(0).setTo(Scalar::all(0));
    result.col(result.cols-1).setTo(Scalar::all(0));
}

string node = "0";

void nodeCallback(const std_msgs::String::ConstPtr& data){
    node = data->data;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "image_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe(BAR_CAM, 1, imageCallback);
    ros::Subscriber scan_sub = nh.subscribe("code/scan",1, &nodeCallback);
	image_transport::Publisher pub = it.advertise(BARCODE_IMG_TOPIC, 1);
	ros::Publisher qr_pub = nh.advertise<std_msgs::String>(QR_TOPIC, 10);
    ros::Publisher bar_pub = nh.advertise<std_msgs::String>(BAR_TOPIC, 10);
    ros::Subscriber node_sub = nh.subscribe("code/node",1, &nodeCallback);
    // Create a zbar reader
    ImageScanner scanner;
    // Configure the reader
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
    while (nh.ok()) {
        // Capture an OpenCV frame
        ros::spinOnce();
        if(scan){
            if(!frame.cols)
            	continue;
            ROS_INFO("Scannig");
            cv::Mat frame_grayscale;
            // Convert to grayscale
            cvtColor(frame, frame_grayscale, COLOR_BGR2GRAY);

            // // Obtain image data
            Mat blurred; double sigma = 1, _threshold = 5, amount = 1;
            GaussianBlur(frame_grayscale, blurred, Size(), sigma, sigma);
            imshow("Blur" ,blurred);
            Mat lowContrastMask; //= abs(frame - blurred) < _threshold;
            Mat sharpened ;//= frame*(1+amount) + blurred*(-amount);
            sharpen(blurred,sharpened);
            imshow("sharp",sharpened);
            // frame.copyTo(sharpened, lowContrastMask);
            //imshow("Pic",frame);
            cvtColor(sharpened, frame_grayscale, COLOR_BGR2GRAY);

            // // then adjust the threshold to actually make it binary
            // Mat binaryMat;
            // threshold(frame_grayscale, binaryMat, LOW_BINARY_THRESHOLD, 255, CV_THRESH_BINARY);
            // process(frame_grayscale);
            // cvtColor(frame, frame_grayscale, CV_BGR2GRAY);
            int width = frame_grayscale.cols;
            int height = frame_grayscale.rows;
            uchar *raw = (uchar *)(frame_grayscale.data);
            imshow("Bin",frame_grayscale);
            // Wrap image data
            Image image(width, height, "Y800", raw, width * height);

            // Scan the image for barcodes
            //int n = scanner.scan(image);
            scanner.scan(image);

            // Extract results
            int counter = 0;
            std_msgs::String qr_msg,bar_msg;
            for (Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {

                if(symbol->get_type_name() == "QR-Code"){
                    string a = node;
                    int y = symbol->get_location_y(0) + symbol->get_location_y(1) + 
                            symbol->get_location_y(1) + symbol->get_location_y(2);
                    y /= 4;
                    if(y < frame_grayscale.rows/2 ){
                        a = a + "dn";
                    }
                    else
                        a = a + "Up";
                    a = a + symbol->get_data();
                    qr_msg.data = a;
                	qr_pub.publish(qr_msg);
                }
                else{
                	bar_msg.data = symbol->get_data();
                	bar_pub.publish(bar_msg);
                }
                if (symbol->get_location_size() == 4) {
                    //rectangle(frame, Rect(symbol->get_location_x(i), symbol->get_location_y(i), 10, 10), Scalar(0, 255, 0));
                    line(frame, Point(symbol->get_location_x(0), symbol->get_location_y(0)), Point(symbol->get_location_x(1), symbol->get_location_y(1)), Scalar(0, 255, 0), 2, 8, 0);
                    line(frame, Point(symbol->get_location_x(1), symbol->get_location_y(1)), Point(symbol->get_location_x(2), symbol->get_location_y(2)), Scalar(0, 255, 0), 2, 8, 0);
                    line(frame, Point(symbol->get_location_x(2), symbol->get_location_y(2)), Point(symbol->get_location_x(3), symbol->get_location_y(3)), Scalar(0, 255, 0), 2, 8, 0);
                    line(frame, Point(symbol->get_location_x(3), symbol->get_location_y(3)), Point(symbol->get_location_x(0), symbol->get_location_y(0)), Scalar(0, 255, 0), 2, 8, 0);
                }
                counter++;
            }
            std_msgs::Header header;
            header.seq = counter; 
            header.stamp = ros::Time::now(); 
            img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, frame);
            img_bridge.toImageMsg(img_msg); 
            //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(img_msg);
            image.set_data(NULL, 0);
        }
        else{
            ROS_INFO("Wait");
        }

    }

    return 0;
}
