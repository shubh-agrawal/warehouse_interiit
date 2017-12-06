#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mavros_msgs/Altitude.h>
#include <warehouse_interiit/Line.h>
#include <warehouse_interiit/LineArray.h>

#define IMAGE_HEIGHT 480
#define IMAGE_WIDTH 640
#define WIDTH_X 20
#define MAX_ERROR_X 30
#define MAX_HOVER_COUNT 50

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

    ros::Subscriber alt_sub = nh.subscribe<mavros_msgs::Altitude>
            ("mavros/altitude", 5, alt_cb);
    ros::Subscriber x_sub = nh.subscribe<LineArray>
            ("lines/horizontal", 5, x_cb);
    ros::Subscriber y_sub = nh.subscribe<Line>
            ("lines/vertical", 5, y_cb);

    ros::Rate rate(20);

    std_msgs::String state;
    Line last_y, last_x, line_x;
    bool isHovering = false, isScanning = false;
    float error = 0, difference = WIDTH_X;
    int hover_count = 0;
    ros::Time scanning_start;
    while(ros::ok()){
        ros::spinOnce();

        //Vertical line preprocess
        if(last_y.rho - line_y.rho < 10 && last_y.rho - line_y.rho > -10)
            last_y = line_y;
        y_pub.publish(last_y);

        //Horizontal line preprocess
        if (!isHovering){
            for(int i = 0; i < lines_x.lines.size(); ++i){
                if((IMAGE_HEIGHT/2) - (lines_x.lines[i].rho) < difference)
                    continue;
                else{
                    x_pub.publish(lines_x.lines[i]);
                    error = IMAGE_HEIGHT/2 - lines_x.lines[i].rho;
                    break;
                }
            }
            if(error < MAX_ERROR_X){
                difference = -1*WIDTH_X;
            }
            if(error<WIDTH_X && error>-WIDTH_X)
                ++hover_count;
            if(hover_count > MAX_HOVER_COUNT){
                isHovering = true;
                scanning_start = ros::Time::now();
                difference = WIDTH_X;
            }
        }
        else{
            for(int i = 0; i < lines_x.lines.size(); ++i){
                if(((IMAGE_HEIGHT/2) - (lines_x.lines[i].rho) < difference) &&
                    ((IMAGE_HEIGHT/2) - (lines_x.lines[i].rho) > -1*difference)){
                    x_pub.publish(lines_x.lines[i]);
                    break;
                }
            }
            if(ros::Time::now() - scanning_start < ros::Duration(10.0)){
                ROS_INFO("Scanning");
            }
            else{
                isHovering = false;
                hover_count = 0;
            }
        }

        if(current_alti > 1 && line_y.rho!=0){
            state.data = "Follow";
        }
        else
            state.data = "Takeoff";
        state_pub.publish(state);
        rate.sleep();
    }

    return 0;
}
