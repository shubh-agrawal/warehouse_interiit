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

#define MAX_VELOCITY 0.15 //Maximum vy and vx value
#define PI 3.14159

#define kpH 0.5
// #define kiH 0.002
// #define kdH 1


// #define kpX 0.0006
// #define kiX 0.00000001
// #define kdX 0.1


//stabilize at differemt setpoint
// #define kpX 0.0006
// #define kiX 0.000003
// #define kdX 0.2

#define kpX 0.001
#define kiX 0.000
#define kdX 0.06

#define kpY 0.0008
#define kiY 0.0000
#define kdY 0.05

#define kpT 0.04
#define kpTL 0.09

using namespace warehouse_interiit;
using namespace std;

ardrone_autonomy::Navdata navdata;
string current_state; //High Level state. Current Supported state are: Land, Takeoff and Follow
float current_alti;
float set_alt;
Line line_x;
Line line_y;
float current_heading, setpoint_heading, mag_reading, l_setpoint;
std_msgs::Empty empty;

void navdata_cb(const ardrone_autonomy::Navdata::ConstPtr& msg){
    navdata = *msg;
    current_alti = (navdata.altd)/1000.0;
}

void x_cb(const Line::ConstPtr& msg){
    line_x = *msg;
}

void y_cb(const Line::ConstPtr& msg){
    line_y = *msg;
}

void yaw_set_cb(const std_msgs::Float32::ConstPtr& msg){
    setpoint_heading = msg->data;
}
void yaw_cb(const std_msgs::Float32::ConstPtr& msg){
    current_heading = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "contvyer_node");
    ros::NodeHandle nh;

    ros::Subscriber navdata_sub = nh.subscribe<ardrone_autonomy::Navdata>
            ("ardrone/navdata", 10, navdata_cb);
    ros::Subscriber x_sub = nh.subscribe<Line>
            ("feedback/vertical", 5, x_cb);
    ros::Subscriber y_sub = nh.subscribe<Line>
            ("feedback/horizontal", 5, y_cb);
    ros::Subscriber yaw_set_sub = nh.subscribe<std_msgs::Float32>
            ("/yaw_setpoint", 5, yaw_set_cb);
    ros::Subscriber yaw_sub = nh.subscribe<std_msgs::Float32>
            ("/heading", 5, yaw_cb);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>
            ("/cmd_vel/strategy", 10);
    ros::Publisher takeoff_pub = nh.advertise<std_msgs::Empty>
            ("ardrone/takeoff", 10);
    ros::Publisher land_pub = nh.advertise<std_msgs::Empty>
            ("ardrone/land", 10);
    ros::ServiceClient flattrim = nh.serviceClient<std_srvs::Empty>("/ardrone/flattrim");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(30.0);

    current_state = "Hovering";

    geometry_msgs::Twist vel;
    float vx = 0.0, vy = 0.0, vz = 0.0, angz = 0.0;
    vel.linear.x = vx;
    vel.linear.y = vy;
    vel.linear.z = vz;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = angz;

    float errorH = 0, sum_errorH = 0, last_errorH = 0,
          errorX = 0, sum_errorX = 0, last_errorX = 0, error_diffX,
          errorY = 0, sum_errorY = 0, last_errorY = 0, error_diffY,
          errorT = 0, sum_errorT = 0, last_errorT = 0;

    std_srvs::Empty emp;
    flattrim.call(emp);
    ros::Time start = ros::Time::now(), cur;
    bool isTookOff = false, heightAchieved = false, nodeReached = false, turning = false;
    line_x.rho = 180;
    while(ros::ok()){
        ros::spinOnce();
        if(navdata.batteryPercent > 40){
            if(ros::Time::now() - start < ros::Duration(0.5) && !isTookOff){
                takeoff_pub.publish(empty);
            }
            if(ros::Time::now() - start < ros::Duration(50.0)){
            	// if(!nodeReached){
	            	if(isTookOff){
                        if(setpoint_heading == 0.0){
    		                errorX = 360.0/2 - line_x.rho;
    		                error_diffX = errorX-last_errorX;
    		                if((errorX < 20 && errorX > -20) && (error_diffX < 20 && error_diffX > -20))
    		                    error_diffX = 0;
    		                vx = kpX*errorX + kdX*error_diffX;
    		                if(vx > MAX_VELOCITY)
    		                    vx = MAX_VELOCITY;
    		                else if(vx < -1*MAX_VELOCITY)
    		                    vx = -1*MAX_VELOCITY;
    		                // ROS_INFO("vx %f \t errorX %f \t p %f \t d %f", vx, errorX, kpX*errorX, kdX*error_diffX);
    		                last_errorX = errorX;

    		                errorY = 640.0/2 - line_y.rho;
    		                error_diffY = errorY - last_errorY;
    		                if((errorY < 20 && errorY > -20) && (error_diffY < 20 && error_diffY > -20))
    		                    error_diffY = 0;
    		                vy = -1*(kpY*errorY + kdY*error_diffY);
    		                if(vy > MAX_VELOCITY)
    		                    vy = MAX_VELOCITY;
    		                else if(vy < -1*MAX_VELOCITY)
    		                    vy = -1*MAX_VELOCITY;
    		                // ROS_INFO("vy %f \t errorY %f \t p %f \t d %f", vy, errorY, kpY*errorY, kdY*error_diffY);
    		                last_errorY = errorY;

                            errorT = setpoint_heading - current_heading;
                            angz = kpT*errorT;
                            if(angz > 1.0)
                                angz = 1.0;
                            else if(angz < -1.0)
                                angz = -1.0;
                        }
                        else{
                            errorT = setpoint_heading - current_heading;
                            angz = kpTL*errorT;
                            if(angz > 1.0)
                                angz = 1.0;
                            else if(angz < -1.0)
                                angz = -1.0;
                        }
					}
	                errorH = 1.5 - current_alti;
	                vz = kpH*errorH;
	                if(vz > 1.0)
	                    vz = 1.0;
	                else if(vz < -1.0)
	                    vz = -1.0;
	                if(abs(errorH) < 0.1){
	                    vz = 0.0;
	                }
	                if(errorH < 0.3)
	                	isTookOff = true;
	            // }
	            // ROS_INFO("X %f Y %f H %f", errorX, errorY, errorH);
             //    if(abs(errorX) < 15 && abs(errorY) < 15 && abs(errorH) < 0.15){
             //    	vx = 0.0;
             //    	vy = 0.0;
             //    	vz = 0.0;
             //    	angz = 0.0;
             //    	nodeReached = true;
             //    	ROS_INFO("Hovering");
             //    }

            }
            else if(ros::Time::now() - start >= ros::Duration(50.0) && isTookOff){
                land_pub.publish(empty);
                vy = 0.0;
            }
            vel.linear.x = vx;
            vel.linear.y = vy;
            vel.linear.z = vz;
            vel.angular.z = angz;
            vel_pub.publish(vel);
        }
        else{
            land_pub.publish(empty);
            ROS_INFO("Battery Low");
        }
        rate.sleep();
    }

    return 0;
}
