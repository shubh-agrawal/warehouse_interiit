#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>

#include <ardrone_autonomy/Navdata.h>

#include <tf/transform_datatypes.h>

#include <warehouse_interiit/Line.h>
#include <warehouse_interiit/LineArray.h>
#include <string.h>

#define MAX_VELOCITY 1 //Maximum vy and vx value
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "contvyer_node");
    ros::NodeHandle nh;

    ros::Subscriber navdata_sub = nh.subscribe<ardrone_autonomy::Navdata>
            ("ardrone/navdata", 10, navdata_cb);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>
            ("/cmd_vel", 10);
    ros::Publisher takeoff_pub = nh.advertise<std_msgs::Empty>
            ("ardrone/takeoff", 10);\
    ros::Publisher land_pub = nh.advertise<std_msgs::Empty>
            ("ardrone/land", 10);
    ros::ServiceClient flattrim = nh.serviceClient<std_srvs::Empty>("/ardrone/flattrim");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    current_state = "Hovering";

    geometry_msgs::Twist vel;
    float vx = 0.0, vy = 0.0, vz = 0.0, angz = 0.0;
    vel.linear.x = vx;
    vel.linear.y = vy;
    vel.linear.z = vz;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = angz;
    std_srvs::Empty emp;
    flattrim.call(emp);
    ros::Time start = ros::Time::now(), cur;
    bool isTookOff = false;
    while(ros::ok()){
        ros::spinOnce();
        if(ros::Time::now() - start < ros::Duration(0.5)){
            takeoff_pub.publish(empty);
        }
        // else if(ros::Time::now() - start < ros::Duration(1.5)){
        //     vx = 0.5;
        // }
        if(navdata.state == 4 && !isTookOff){
            isTookOff = true;
            start = ros::Time::now();
            vy = -0.5;
        }
        if(ros::Time::now() - start > ros::Duration(3.0) && isTookOff){
            land_pub.publish(empty);
        }
        vel.linear.y = vy;
        vel_pub.publish(vel);
        rate.sleep();
    }

    return 0;
}
