#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <ardrone_autonomy/Navdata.h>
#include <ardrone_autonomy/CamSelect.h>
#include <warehouse_interiit/Line.h>
#include <warehouse_interiit/LineArray.h>
#include "Config.h"
#include <queue>

#define IMAGE_HEIGHT 640
#define IMAGE_WIDTH 360
#define WIDTH_X 200
#define MAX_ERROR_X 250
#define MAX_HOVER_COUNT 10
#define MIN_HEIGHT 1.0
#define MAX_HEIGHT 1.8
#define CAMERA_VAR_X 30
#define CAMERA_VAR_Y 30
#define MOTION_VAR_X 30
#define MOTION_VAR_Y 30
#define GAMMA 5
#define PI 3.14159
using namespace warehouse_interiit;



float current_alti, mag_heading, line_heading;
Line line_y;
LineArray lines_x;

std::queue<float> xs, ys, thetas;

float x, y, theta;
ardrone_autonomy::Navdata navdata;

void navdata_cb(const ardrone_autonomy::Navdata::ConstPtr& msg){
    navdata = *msg;
    current_alti = (navdata.altd)/1000.0;
    mag_heading = navdata.rotZ;
}

float getAverage(float newVal, std::queue<float>& q, float oldVal){
    q.push(newVal);
    if(q.size() > GAMMA){
        float last = q.front();
        q.pop();
        oldVal -= last/GAMMA;
        return oldVal += newVal/q.size();
    }
    else{
        return (oldVal*(q.size()-1) + newVal)/q.size();
    }
}

void x_cb(const LineArray::ConstPtr& msg){
    lines_x = *msg;
}

void y_cb(const Line::ConstPtr& msg){
    line_y = *msg;
    y = getAverage(abs(line_y.rho), ys, y);
    line_y.rho = y;
    line_heading = (line_y.theta)*180/PI;
    if(line_heading > 130.0)
        line_heading = line_heading - 180;
    theta = getAverage(line_heading, thetas, theta);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "strategy_node");
    ros::NodeHandle nh;

    ros::Publisher state_pub = nh.advertise<std_msgs::String>("/state", 10);
    ros::Publisher x_pub = nh.advertise<Line>("feedback/horizontal", 10);
    ros::Publisher y_pub = nh.advertise<Line>("feedback/vertical", 10);
    ros::Publisher alt_set_pub = nh.advertise<std_msgs::Float32>("altitude_setpoint", 10);
    ros::Publisher yaw_set_pub = nh.advertise<std_msgs::Float32>("yaw_setpoint", 10);
    ros::Publisher yaw_pub = nh.advertise<std_msgs::Float32>("heading", 10);
    ros::Publisher barcode_scan = nh.advertise<std_msgs::Bool>("code/scan", 100);
    ros::Subscriber navdata_sub = nh.subscribe<ardrone_autonomy::Navdata>
            ("ardrone/navdata", 5, navdata_cb);
    ros::Subscriber x_sub = nh.subscribe<LineArray>
            ("lines/horizontal", 5, x_cb);
    ros::Subscriber y_sub = nh.subscribe<Line>
            ("lines/vertical", 5, y_cb);

    ros::ServiceClient camToggle = nh.serviceClient<ardrone_autonomy::CamSelect>("/ardrone/setcamchannel");

    ros::Rate rate(30);
    Line last_y, last_x, line_x;
    std_msgs::Float32 alt_set, yaw_setpoint, current_heading;
    alt_set.data = MIN_HEIGHT;
    bool isHovering = false, isScanning = false, turnFlag = false, yawSet = false;
    std_msgs::String state;
    float error = 999999, difference = WIDTH_X;
    int hover_count = 0;
    ros::Time scanning_start;
    float last_rho;
    y = 0.0, x = 0.0, theta = 0.0;
    std_msgs::Bool scan;
    ardrone_autonomy::CamSelect cam;
    cam.request.channel = 1;
    camToggle.call(cam);
    int i = 0;
    int node_num = 0;
    bool stationary = false;
    while(ros::ok()){
        ros::spinOnce();

        //Vertical line preprocess
        y_pub.publish(line_y);

        //Horizontal line preprocess
        if (!isHovering){
            for(int i = 0; i < lines_x.lines.size(); ++i){
                if((IMAGE_HEIGHT/2) - (lines_x.lines[i].rho) < difference)
                    continue;
                else{
                    if(lines_x.lines[i].rho >= 1.0){
                        x = getAverage(abs(lines_x.lines[i].rho), xs, x);
                        lines_x.lines[i].rho = x;
                    }
                    x_pub.publish(lines_x.lines[i]);
                    last_rho = lines_x.lines[i].rho;
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
                scanning_start = ros::Time::now();
                difference = WIDTH_X;
                node_num += 1;
                ROS_INFO("node number %d", node_num);
            }
            state.data = "Follow";
        }
        else{
            // state.data = "Hovering";
            Line temp;
            for(int i = 0; i < lines_x.lines.size(); ++i){
                float rho = lines_x.lines[i].rho;
                if(fabs(last_rho - rho) < 50.0){
                    temp = lines_x.lines[i];
                    if(abs(last_rho - temp.rho) < 5)
                        stationary = true;
                    else
                        stationary = false;
                    last_rho = temp.rho;
                    break;
                }
            }
            x_pub.publish(temp);
            if(node_num == 2 && stationary){
                state.data = "Turning";
                yaw_setpoint.data = mag_heading - 90.0;
                scan.data = false;
                isHovering = false;
                hover_count = 0;
                difference = WIDTH_X;
                alt_set.data = MIN_HEIGHT;
                while(abs(yaw_setpoint.data - mag_heading) > 20){
                    current_heading.data = mag_heading;
                    yaw_pub.publish(current_heading);
                    yaw_set_pub.publish(yaw_setpoint);
                    state_pub.publish(state);
                    ros::spinOnce();
                    rate.sleep();
                }
                continue;
            }
            // else{
            //     if(ros::Time::now() - scanning_start < ros::Duration(10.0)){
            //         alt_set.data = MIN_HEIGHT;
            //         scan.data = true;
            //         barcode_scan.publish(scan);
            //         ROS_INFO("Scanning");
            //     }
            //     else if(ros::Time::now() - scanning_start < ros::Duration(10.0 + (MAX_HEIGHT-MIN_HEIGHT)*10)){
            //         if(alt_set.data < MAX_HEIGHT)
            //             alt_set.data += 0.01;
            //         scan.data = true;
            //         barcode_scan.publish(scan);
            //         ROS_INFO("Scanning");
            //     }
            //     else if(ros::Time::now() - scanning_start < ros::Duration(10.0 + (MAX_HEIGHT-MIN_HEIGHT)*20)){
            //         if(alt_set.data > MIN_HEIGHT)
            //             alt_set.data -= 0.01;
            //         scan.data = true;
            //         barcode_scan.publish(scan);
            //         ROS_INFO("Scanning");
            //     }
            //     else if(ros::Time::now() - scanning_start < ros::Duration(10.0 + (MAX_HEIGHT-MIN_HEIGHT)*25)){
            //         alt_set.data = MIN_HEIGHT;
            //         scan.data = true;
            //         barcode_scan.publish(scan);
            //         ROS_INFO("Scanning");
            //     }
            //     else{
            //         alt_set.data = MIN_HEIGHT;
            //         scan.data = false;
            //         isHovering = false;
            //         hover_count = 0;
            //         difference = WIDTH_X;
            //     }
            // }
            if(ros::Time::now() - scanning_start > ros::Duration(15.0)){
                scan.data = false;
                isHovering = false;
                hover_count = 0;
                difference = WIDTH_X;
                // cam.request.channel = 1;
                // camToggle.call(cam);
            }
        }
        state_pub.publish(state);
        current_heading.data = line_heading;
        yaw_pub.publish(current_heading);
        yaw_setpoint.data = 0.0;
        // alt_set_pub.publish(alt_set);
        yaw_set_pub.publish(yaw_setpoint);
        rate.sleep();
    }

    return 0;
}
