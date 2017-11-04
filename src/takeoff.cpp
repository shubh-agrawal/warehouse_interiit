/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>
#include <std_msgs/Float64.h>
#include <iostream>

#define kp 0.11
#define ki 0.0001
#define kd 1

mavros_msgs::State current_state;
mavros_msgs::Altitude alti;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void alt_cb(const mavros_msgs::Altitude::ConstPtr& msg){
    alti = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber alt_sub = nh.subscribe<mavros_msgs::Altitude>
            ("mavros/altitude", 5, alt_cb);
    ros::Publisher att_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_attitude/attitude", 10);
    ros::Publisher thrst_pub = nh.advertise<std_msgs::Float64>
            ("mavros/setpoint_attitude/att_throttle", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x =  0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w =  1;

    std_msgs::Float64 thrust;
    thrust.data = 0.0;
    float set_alt = 2.0;
    float cur_alt = 0.0;
    //send a few setpoints before starting
    // for(int i = 100; ros::ok() && i > 0; --i){
    //     local_pos_pub.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    float error = 0, sum_error = 0, last_error = 0;
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        att_pub.publish(pose);
        cur_alt = alti.local;
        error = set_alt - cur_alt;
        sum_error += error;
        std::cout << kp*error << "  " << kd*(error-last_error) << std::endl;
        thrust.data = 0.43 + kp*error + kd*(error-last_error) + ki*sum_error;
        if(thrust.data > 1)
        	thrust.data = 1;
        else if(thrust.data < 0)
        	thrust.data = 0;
        thrst_pub.publish(thrust);
        last_error = error;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
