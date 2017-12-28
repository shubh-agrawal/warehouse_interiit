#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include <ardrone_autonomy/Navdata.h>

#include <tf/transform_datatypes.h>

#include <warehouse_interiit/Line.h>
#include <warehouse_interiit/LineArray.h>
#include <string.h>

#define MAX_VELOCITY 0.4 //Maximum vy and vx value
#define PI 3.14159

// #define kpH 0.115
// #define kiH 0.002
// #define kdH 1

// #define kpR 0.001
// #define kiR 0.0
#define kdR 0.0

// #define kpP 0.001
#define kiP 0.0
#define kdP 0.0

using namespace warehouse_interiit;
using namespace std;

ardrone_autonomy::Navdata navdata;
string current_state; //High Level state. Current Supported state are: Land, Takeoff and Follow
float current_alti;
float set_alt;
Line line_x;
Line line_y;
float current_heading;
std_msgs::Empty empty;

void navdata_cb(const ardrone_autonomy::Navdata::ConstPtr& msg){
    navdata = *msg;
    current_alti = (navdata.altd)/1000.0;
    current_heading = navdata.rotZ;
}

void state_cb(const std_msgs::String::ConstPtr& msg){
    current_state = msg->data;
}

void alt_set_cb(const std_msgs::Float32::ConstPtr& msg){
     set_alt = msg->data;
}

void x_cb(const Line::ConstPtr& msg){
    line_x = *msg;
}

void y_cb(const Line::ConstPtr& msg){
    line_y = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "contvyer_node");
    ros::NodeHandle nh;

    ros::Subscriber navdata_sub = nh.subscribe<ardrone_autonomy::Navdata>
            ("ardrone/navdata", 10, navdata_cb);

    ros::Subscriber state_sub = nh.subscribe<std_msgs::String>
            ("state", 5, state_cb);
    // ros::Subscriber alt_sub = nh.subscribe<std_msgs::Float32>
    //         ("/altitude", 5, alt_cb);
    ros::Subscriber alt_set_sub = nh.subscribe<std_msgs::Float32>
            ("altitude_setpoint", 5, alt_set_cb);
    ros::Subscriber x_sub = nh.subscribe<Line>
            ("feedback/horizontal", 5, x_cb);
    ros::Subscriber y_sub = nh.subscribe<Line>
            ("feedback/vertical", 5, y_cb);
    // ros::Subscriber yaw_sub = nh.subscribe<std_msgs::Float64>
    //         ("/mavros/global_position/compass_hdg", 5, heading_cb);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>
            ("ardrone/cmd_vel", 10);
    ros::Publisher takeoff_pub = nh.advertise<std_msgs::Empty>
            ("ardrone/takeoff", 10);\
    ros::Publisher land_pub = nh.advertise<std_msgs::Empty>
            ("ardrone/land", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    double kpH, kiH, kdH, kpX, kiX, kdX, kpY, kiY, kdY, kpT, kiT, kdT;

    if (nh.getParam("kp_height", kpH)){
      ROS_INFO("Got param: %f", kpH);
    }
    else{
      ROS_ERROR("Failed to get param 'kp_height'");
    }
    if (nh.getParam("ki_height", kiH)){
      ROS_INFO("Got param: %f", kiH);
    }
    else{
      ROS_ERROR("Failed to get param 'ki_height'");
    }
    if (nh.getParam("kd_height", kdH)){
      ROS_INFO("Got param: %f", kdH);
    }
    else{
      ROS_ERROR("Failed to get param 'kd_height'");
    }
    if (nh.getParam("kp_vy", kpY)){
      ROS_INFO("Got param: %f", kpY);
    }
    else{
      ROS_ERROR("Failed to get param 'kp_vy'");
    }
    if (nh.getParam("kp_vx", kpX)){
      ROS_INFO("Got param: %f", kpX);
    }
    else{
      ROS_ERROR("Failed to get param 'kp_vx'");
    }

    // wait to takeoff
    for(int i = 0; ros::ok() && i < 10; ++i){
        ros::spinOnce();
        takeoff_pub.publish(empty);
        rate.sleep();
    }

    current_state = "Hovering";

    geometry_msgs::Twist vel;
    float vx = 0.0, vy = 0.0, vz = 0.0, angz = 0.0;
    vel.linear.x = vx;
    vel.linear.y = vy;
    vel.linear.z = vz;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = angz;

    ros::Time last_request = ros::Time::now();
    float errorH = 0, sum_errorH = 0, last_errorH = 0,
          errorX = 0, sum_errorX = 0, last_errorX = 0,
          errorY = 0, sum_errorY = 0, last_errorY = 0,
          errorT = 0, sum_errorT = 0, last_errorT = 0;
    float initial_yaw = current_heading;
    float yaw = initial_yaw;
    while(ros::ok()){
        ros::spinOnce();

        if(current_state == "Turn_Left"){
            yaw = initial_yaw + 90.0;
            if(yaw > 180)
                yaw = yaw - 180;
        }
        else if(current_state == "Turn_Right"){
            yaw = initial_yaw - 90;
            if(yaw < -180)
                yaw = yaw + 180.0;
        }

        // Follow mode: Follows line or hover at a node.
        if(current_state == "Follow"){
            errorX = 240.0 - line_x.rho;
            if(errorX < 50 && errorX > -50)
                sum_errorX += errorX;
            vx = kpX*errorX + kiX*sum_errorX + kdX*(errorX-last_errorX);
            if(vx > MAX_VELOCITY)
                vx = MAX_VELOCITY;
            else if(vx < -1*MAX_VELOCITY)
                vx = -1*MAX_VELOCITY;
            last_errorX = errorX;
            errorY = 320.0 - line_y.rho;
            vy = -1*(kpY*errorY + kdY*(errorY-last_errorY));
            if(vy > MAX_VELOCITY)
                vy = MAX_VELOCITY;
            else if(vy < -1*MAX_VELOCITY)
                vy = -1*MAX_VELOCITY;
            last_errorY = errorY;
            angz = 0.0;
        }
        // Turning...
        else{
            vy = 0.0;
            vx = 0.0;
            angz = 0.0;
            errorT = yaw - current_heading;
            sum_errorT += errorT;
            angz = kpT*errorT + kiT*sum_errorT + kdT*(errorT-last_errorT);
            if(angz > MAX_VELOCITY)
                angz = MAX_VELOCITY;
            else if(angz < -1*MAX_VELOCITY)
                angz = -1*MAX_VELOCITY;
            last_errorT = errorT;
        }

        // Takeoff
        if(current_state == "Takeoff"){
            takeoff_pub.publish(empty);
            rate.sleep();
            continue;
        }
        // Land Mode
        else if(current_state == "Land"){
            land_pub.publish(empty);
            rate.sleep();
            continue;
        }
        // Height Control
        else{
            errorH = set_alt - current_alti;
            if(current_alti >= 0.25)
                sum_errorH += errorH;
            vz = kpH*errorH + kdH*(errorH-last_errorH) + kiH*sum_errorH;
            if(vz > MAX_VELOCITY)
                vz = MAX_VELOCITY;
            else if(vz < -1*MAX_VELOCITY)
                vz = -1*MAX_VELOCITY;
            last_errorH = errorH;
        }
        //ROS_INFO("vy: %f vx: %f Yaw: %f", vy, vx, yaw*180/PI);
        vel.linear.x = vx;
        vel.linear.y = vy;
        vel.linear.z = vz;
        vel.angular.x = 0.0;
        vel.angular.y = 0.0;
        vel.angular.z = angz;
        vel_pub.publish(vel);
        rate.sleep();
    }

    return 0;
}
