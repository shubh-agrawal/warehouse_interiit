#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

ButtonAuto = 4
ButtonManual = 5

auto_flag = 1

manual_cmd_vel = Twist()
strategy_cmd_vel = Twist()

def joystick_callback(joy_msg):
    global auto_flag
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", joy_msg.data)
    if (joy_msg.buttons[ButtonAuto] == 1 and joy_msg.buttons[ButtonManual] == 0):
        auto_flag = 1
    elif (joy_msg.buttons[ButtonAuto] == 0 and joy_msg.buttons[ButtonManual] == 1):
        auto_flag = 0
    else:
        pass
    rospy.loginfo("%d", auto_flag)

def manual_callback(manual_msg):
    global manual_cmd_vel
    manual_cmd_vel = manual_msg

    if (auto_flag == 0):
        main_pub.publish(manual_cmd_vel)

def strategy_callback(strategy_msg):
    global strategy_cmd_vel
    global auto_flag

    strategy_cmd_vel = strategy_msg

    if (auto_flag == 1):
        main_pub.publish(strategy_cmd_vel)


if __name__ == '__main__':

        rospy.init_node('mode_switcher', anonymous=True)
        rospy.Subscriber("/joy", Joy, joystick_callback)
        rospy.Subscriber("/cmd_vel/manual", Twist, manual_callback)
        rospy.Subscriber("/cmd_vel/strategy", Twist, strategy_callback)
        main_pub = rospy.Publisher('/cmd_vel',Twist)
        rospy.spin()





