/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/Thrust.h>

#define kp 0.115
#define ki 0.002
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
    //ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //        ("mavros/setpoint_position/local", 10);
    ros::Publisher att_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_attitude/target_attitude", 10);
    ros::Publisher thrust_pub = nh.advertise<mavros_msgs::Thrust>
            ("mavros/setpoint_attitude/thrust", 10);
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

    mavros_msgs::Thrust thrust;
    thrust.thrust = 0.0;
    float set_alt = 2.0;
    float cur_alt = 0.0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
	pose.header.stamp = ros::Time::now();
        att_pub.publish(pose);
	thrust.header.stamp = ros::Time::now();
	thrust_pub.publish(thrust);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    float error = 0, sum_error = 0, last_error = 0;
    while(ros::ok()){
	ros::spinOnce();
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
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
	if(current_state.armed){
		pose.header.stamp = ros::Time::now();
        	att_pub.publish(pose);
        	cur_alt = alti.local;
        	error = set_alt - cur_alt;
        	sum_error += error;
        //std::cout << kp*error << "  " << kd*(error-last_error) << std::endl;
        	thrust.thrust = 0.5 + kp*error + kd*(error-last_error) + ki*sum_error;
        	if(thrust.thrust > 1)
                	thrust.thrust = 1;
        	else if(thrust.thrust < 0)
                	thrust.thrust = 0;
		thrust.header.stamp = ros::Time::now();
        	thrust_pub.publish(thrust);
        	last_error = error;

               	rate.sleep();
	}
    }

    return 0;
}
