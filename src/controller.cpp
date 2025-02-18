#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/Thrust.h>

#include <tf/transform_datatypes.h>

#include <warehouse_interiit/Line.h>
#include <warehouse_interiit/LineArray.h>
#include <string.h>

#define MAX_ANGLE 0.1 //Maximum roll and pitch value
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

mavros_msgs::State current_fcstate; //Flight Controller state
string current_state; //High Level state. Current Supported state are: Land, Takeoff and Follow
float current_alti;
float set_alt;
Line line_x;
Line line_y;
float current_heading;

void fcstate_cb(const mavros_msgs::State::ConstPtr& msg){
    current_fcstate = *msg;
}

void state_cb(const std_msgs::String::ConstPtr& msg){
    current_state = msg->data;
}

void alt_cb(const std_msgs::Float32::ConstPtr& msg){
     current_alti = msg->data;
}

void alt_set_cb(const std_msgs::Float32::ConstPtr& msg){
     set_alt = msg->data;
}

void heading_cb(const std_msgs::Float64::ConstPtr& msg){
    current_heading = (msg->data)*PI/180.0;
}

void x_cb(const Line::ConstPtr& msg){
    line_x = *msg;
}

void y_cb(const Line::ConstPtr& msg){
    line_y = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh;

    ros::Subscriber fcstate_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, fcstate_cb);

    ros::Subscriber state_sub = nh.subscribe<std_msgs::String>
            ("state", 5, state_cb);
    ros::Subscriber alt_sub = nh.subscribe<std_msgs::Float32>
            ("/altitude", 5, alt_cb);
    ros::Subscriber alt_set_sub = nh.subscribe<std_msgs::Float32>
            ("altitude_setpoint", 5, alt_set_cb);
    ros::Subscriber x_sub = nh.subscribe<Line>
            ("feedback/horizontal", 5, x_cb);
    ros::Subscriber y_sub = nh.subscribe<Line>
            ("feedback/vertical", 5, y_cb);
    ros::Subscriber yaw_sub = nh.subscribe<std_msgs::Float64>
            ("/mavros/global_position/compass_hdg", 5, heading_cb);

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

    double kpH, kiH, kdH, kpP, kpR, min_thrust = 0.2;

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
    if (nh.getParam("kp_roll", kpR)){
      ROS_INFO("Got param: %f", kpR);
    }
    else{
      ROS_ERROR("Failed to get param 'kp_roll'");
    }
    if (nh.getParam("kp_pitch", kpP)){
      ROS_INFO("Got param: %f", kpP);
    }
    else{
      ROS_ERROR("Failed to get param 'kp_pitch'");
    }
    if (nh.getParam("min_thrust", min_thrust)){
      ROS_INFO("Got param: %f", min_thrust);
    }
    else{
      ROS_ERROR("Failed to get param 'kp_pitch'");
    }

    // wait for FCU connection
    while(ros::ok() && current_fcstate.connected){
        ros::spinOnce();
        rate.sleep();
    }

    current_state = "Land";

    geometry_msgs::PoseStamped pose;
    tf::Quaternion q;
    float roll = 0, pitch = 0, initial_yaw = current_heading, yaw;
    pose.pose.position.z = 0.0;
    q.setRPY(roll, pitch, yaw);
    tf::quaternionTFToMsg(q, pose.pose.orientation);

    mavros_msgs::Thrust thrust;
    thrust.thrust = 0.0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        pose.header.stamp = ros::Time::now();
        thrust.header.stamp = ros::Time::now();
        yaw = current_heading;
        q.setRPY(roll, pitch, yaw);
        tf::quaternionTFToMsg(q, pose.pose.orientation);
        att_pub.publish(pose);
        thrust_pub.publish(thrust);
        ros::spinOnce();
        rate.sleep();
    }

    // mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "OFFBOARD";

    // mavros_msgs::CommandBool arm_cmd;
    // arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    float errorH = 0, sum_errorH = 0, last_errorH = 0,
          errorR = 0, sum_errorR = 0, last_errorR = 0,
          errorP = 0, sum_errorP = 0, last_errorP = 0;
    initial_yaw = current_heading;
    yaw = initial_yaw;
    while(ros::ok()){
        ros::spinOnce();
        // if( current_fcstate.mode != "OFFBOARD" &&
        //     (ros::Time::now() - last_request > ros::Duration(5.0))){
        //     if( set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.mode_sent){
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now();
        // } else {
        //     if( !current_fcstate.armed &&
        //         (ros::Time::now() - last_request > ros::Duration(5.0))){
        //         if( arming_client.call(arm_cmd) &&
        //             arm_cmd.response.success){
        //             ROS_INFO("Vehicle armed");
        //         }
        //         last_request = ros::Time::now();
        //     }
        // }


        if(current_state == "Turn_Left"){
            yaw = initial_yaw + PI/2;
            if(yaw > 2*PI)
                yaw = yaw - 2*PI;
        }
        else if(current_state == "Turn_Right"){
            yaw = initial_yaw - PI/2;
            if(yaw < 0)
                yaw = yaw + 2*PI;
        }

        // Follow mode: Follows line or hover at a node.
        if(current_state == "Follow"){
            errorP = 240.0 - line_x.rho;
            if(errorP < 50 && errorP > -50)
                sum_errorP += errorP;
            pitch = kpP*errorP + kiP*sum_errorP + kdP*(errorP-last_errorP);
            if(pitch > MAX_ANGLE)
                pitch = MAX_ANGLE;
            else if(pitch < -1*MAX_ANGLE)
                pitch = -1*MAX_ANGLE;
            last_errorP = errorP;
            errorR = 320.0 - line_y.rho;
            roll = -1*(kpR*errorR + kdR*(errorR-last_errorR));
            if(roll > MAX_ANGLE)
                roll = MAX_ANGLE;
            else if(roll < -1*MAX_ANGLE)
                roll = -1*MAX_ANGLE;
            last_errorR = errorR;
            q.setRPY(roll, pitch, yaw);
            tf::quaternionTFToMsg(q, pose.pose.orientation);
        }
        else{
            roll = 0.0;
            pitch = 0.0;
            q.setRPY(roll, pitch, yaw);
            tf::quaternionTFToMsg(q, pose.pose.orientation);
        }
        // Runs in both Takeoff and Follow mode hor height control
        if(current_state != "Land"){
            errorH = set_alt - current_alti;
            if(current_alti >= 0.25)
                sum_errorH += errorH;
            thrust.thrust = min_thrust + kpH*errorH + kdH*(errorH-last_errorH) + kiH*sum_errorH;
            if(thrust.thrust > min_thrust + 0.2)
                thrust.thrust = min_thrust + 0.2;
            else if(thrust.thrust < min_thrust - 0.1)
                thrust.thrust = min_thrust - 0.1;
            thrust_pub.publish(thrust);
            last_errorH = errorH;
        }
        // Land Mode
        else{
            thrust.thrust = 0.2;
        }
        //ROS_INFO("Roll: %f Pitch: %f Yaw: %f", roll, pitch, yaw*180/PI);
        pose.header.stamp = ros::Time::now();
        thrust.header.stamp = ros::Time::now();
        att_pub.publish(pose);
        thrust_pub.publish(thrust);
        rate.sleep();
    }

    return 0;
}
