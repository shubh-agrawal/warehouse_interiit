#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mavros_msgs/Altitude.h>

float current_alti;
void alt_cb(const mavros_msgs::Altitude::ConstPtr& msg){
    current_alti = msg->local;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "strategy_node");
    ros::NodeHandle nh;
    ros::Publisher state_pub = nh.advertise<std_msgs::String>("state", 10);
    ros::Subscriber alt_sub = nh.subscribe<mavros_msgs::Altitude>
            ("mavros/altitude", 5, alt_cb);
    ros::Rate rate(10);
    std_msgs::String state;
    while(ros::ok()){
        ros::spinOnce();
        if(current_alti > 1 )
            state.data = "Follow";
        else
            state.data = "Takeoff";
        state_pub.publish(state);
        rate.sleep();
    }

    return 0;
}
