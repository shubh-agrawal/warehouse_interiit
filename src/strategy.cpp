#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <mavros_msgs/Altitude.h>
#include <warehouse_interiit/Line.h>
#include <warehouse_interiit/LineArray.h>
#include "Config.h"

#define IMAGE_HEIGHT 480
#define IMAGE_WIDTH 640
#define WIDTH_X 50
#define MAX_ERROR_X 75
#define MAX_HOVER_COUNT 10
#define MIN_HEIGHT 1.0
#define MAX_HEIGHT 1.8

using namespace warehouse_interiit;

float current_alti;
Line line_y;
LineArray lines_x;

void alt_cb(const mavros_msgs::Altitude::ConstPtr& msg){
    current_alti = msg->local;
}
void x_cb(const LineArray::ConstPtr& msg){
    lines_x = *msg;
}

void y_cb(const Line::ConstPtr& msg){
    line_y = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "strategy_node");
    ros::NodeHandle nh;

    ros::Publisher state_pub = nh.advertise<std_msgs::String>("state", 10);
    ros::Publisher x_pub = nh.advertise<Line>("feedback/horizontal", 10);
    ros::Publisher y_pub = nh.advertise<Line>("feedback/vertical", 10);
    ros::Publisher alt_set_pub = nh.advertise<std_msgs::Float32>("altitude_setpoint", 10);
    ros::Publisher barcode_scan = nh.advertise<std_msgs::Bool>("code/scan", 100);
    ros::Subscriber alt_sub = nh.subscribe<mavros_msgs::Altitude>
            ("mavros/altitude", 5, alt_cb);
    ros::Subscriber x_sub = nh.subscribe<LineArray>
            ("lines/horizontal", 5, x_cb);
    ros::Subscriber y_sub = nh.subscribe<Line>
            ("lines/vertical", 5, y_cb);

    ros::Rate rate(20);
    bool first_hover;
    float hover_rho;
    std_msgs::String state;
    Line last_y, last_x, line_x;
    std_msgs::Float32 alt_set;
    alt_set.data = MIN_HEIGHT;
    bool isHovering = false, isScanning = false, turnFlag = false;
    float error = 999999, difference = WIDTH_X;
    int hover_count = 0;
    ros::Time scanning_start;
    float last_rho;
    while(ros::ok()){
        ros::spinOnce();
        //Vertical line preprocess
        // if(last_y.rho - line_y.rho < 10 && last_y.rho - line_y.rho > -10)
        //     last_y = line_y;
        std_msgs::Bool scan;
        scan.data = false;
        barcode_scan.publish(scan);
        y_pub.publish(line_y);

        //Horizontal line preprocess
        if (!isHovering){
            for(int i = 0; i < lines_x.lines.size(); ++i){
                if((IMAGE_HEIGHT/2) - (lines_x.lines[i].rho) < difference)
                    continue;
                else{
                    last_rho = lines_x.lines[i].rho;
                    x_pub.publish(lines_x.lines[i]);
                    error = IMAGE_HEIGHT/2 - last_rho;
                    break;
                }
            }
            ROS_INFO("error: %f, difference: %f", error, difference);
            if(error < MAX_ERROR_X){
                difference = -1*WIDTH_X;
            }
            if(error<WIDTH_X && error>-WIDTH_X)
                ++hover_count;
            if(hover_count > MAX_HOVER_COUNT){
                isHovering = true;
                // first_hover = true;
                scanning_start = ros::Time::now();
                difference = WIDTH_X;
            }
            alt_set.data = MIN_HEIGHT;
        }
        else{
            Line temp;
            for(int i = 0; i < lines_x.lines.size(); ++i){
                float rho = lines_x.lines[i].rho;
                if(fabs(last_rho - rho) < 2*difference){
                    temp = lines_x.lines[i];
                    last_rho = temp.rho;
                    break;
                }
            }
            x_pub.publish(temp);
            if(temp.L){
                state.data = (temp.L > 0)?"Turn_Right":"Turn_Left";
                scan.data = false;
                isHovering = false;
                hover_count = 0;
                difference = WIDTH_X;
                alt_set.data = MIN_HEIGHT;
                for(int i = 0; i < 5; ++i){
                    state_pub.publish(state);
                    alt_set_pub.publish(alt_set);
                    ros::Duration(0.1).sleep();
                }
                continue;
            }
            else{
                if(ros::Time::now() - scanning_start < ros::Duration(10.0)){
                    alt_set.data = MIN_HEIGHT;
                    scan.data = true;
                    barcode_scan.publish(scan);
                    ROS_INFO("Scanning");
                }
                else if(ros::Time::now() - scanning_start < ros::Duration(10.0 + (MAX_HEIGHT-MIN_HEIGHT)*10)){
                    if(alt_set.data < MAX_HEIGHT)
                        alt_set.data += 0.01;
                    scan.data = true;
                    barcode_scan.publish(scan);
                    ROS_INFO("Scanning");
                }
                else if(ros::Time::now() - scanning_start < ros::Duration(10.0 + (MAX_HEIGHT-MIN_HEIGHT)*20)){
                    if(alt_set.data > MIN_HEIGHT)
                        alt_set.data -= 0.01;
                    scan.data = true;
                    barcode_scan.publish(scan);
                    ROS_INFO("Scanning");
                }
                else if(ros::Time::now() - scanning_start < ros::Duration(10.0 + (MAX_HEIGHT-MIN_HEIGHT)*25)){
                    alt_set.data = 0.01;
                    scan.data = true;
                    barcode_scan.publish(scan);
                    ROS_INFO("Scanning");
                }
                else{
                    alt_set.data = MIN_HEIGHT;
                    scan.data = false;
                    isHovering = false;
                    hover_count = 0;
                    difference = WIDTH_X;
                }
            }
        }
        if(current_alti >= (MIN_HEIGHT - 0.2) && line_y.rho!=0){
            state.data = "Follow";
        }
        else
            state.data = "Takeoff";
        state_pub.publish(state);
        alt_set_pub.publish(alt_set);
        rate.sleep();
    }

    return 0;
}
