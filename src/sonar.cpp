#include <iostream>
#include <wiringPi.h>
#include <ros/ros.h>
#include <std_msgs/Float32>
using namespace std;

int main(int argc, char **argv){
    ros::init(argc, argv, "sonar_node");
    wiringPiSetup();
    ros::NodeHandle nh;
    ros::Publisher alt_pub = nh.advertise<std_msgs::Float32>("/altitude", 10);
    unsigned int timeout = 20000;
    int pin = 7;
    unsigned long int starttime, endtime, watchtime, duration;
    std_msgs::Float32 distance;
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
                    goodread=false;
                }
            }
        }

        if(goodread){
            duration = endtime - starttime;
            distance.data = (duration*34.0)/2000.0;
            ROS_INFO("distance: %f", distance.data);
        }
    }

    return 0;
}
