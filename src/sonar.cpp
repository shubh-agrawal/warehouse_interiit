#include <iostream>
#include <wiringPi.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>
using namespace std;


int main(int argc, char **argv){
    ros::init(argc, argv, "sonar_node");
    setenv("WIRINGPI_GPIOMEM", "1", 1);
    wiringPiSetup();
    ros::NodeHandle nh;
    ros::Publisher alt_pub = nh.advertise<std_msgs::Float32>("/altitude", 10);
    ros::Publisher range_pub = nh.advertise<sensor_msgs::Range>
		("/mavros/distance_sensor/sonar_1_sub", 10);
    ros::Rate rate(20);
    unsigned int timeout = 20000;
    int pin = 7;
    unsigned long int starttime, endtime, watchtime, duration;
    std_msgs::Float32 distance;
    sensor_msgs::Range range;
    range.radiation_type  = range.ULTRASOUND;
    range.field_of_view = 0.261799;
    range.min_range = 0.05;
    range.max_range = 3.0;
    while(ros::ok()){
        pinMode(pin, OUTPUT);
        digitalWrite(pin, 0);
        delayMicroseconds(2);
        digitalWrite(pin, 1);
        delayMicroseconds(5);
        digitalWrite(pin, 0);
        pinMode(pin, INPUT);
        bool goodread = true;
        watchtime = micros();
        while(digitalRead(pin) == 0 && goodread){
            starttime = micros();
            if(starttime - watchtime > timeout){
                    goodread = false;
            }
        }

        if(goodread){
            watchtime = micros();
            while(digitalRead(pin) && goodread){
                endtime = micros();
                if(endtime - watchtime > timeout){
                    ROS_INFO("timeout");
                    goodread=false;
                }
            }
        }

        if(goodread){
	    range.header.stamp = ros::Time::now();
            duration = endtime - starttime;
            distance.data = (duration*34.0)/200000.0;
            range.range = distance.data;
            // ROS_INFO("distance: %f", distance.data);
            alt_pub.publish(distance);
            range_pub.publish(range);
        }
    ros::spinOnce();
    rate.sleep();
    }

    return 0;
}
