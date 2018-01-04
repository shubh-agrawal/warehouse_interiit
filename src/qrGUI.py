#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import texttable
import os

qrdata = []
string = ""

def handy(qr): 
    os.system('cls' if os.name == 'nt' else 'clear')
    table = texttable.Texttable()    
    
    for q in qr :
        node , s = q[:1] , q[1:];
        pos = s[:2]
        data = s[2:]
        if pos == "dn" :
            pos = "0"
	else :
	    pos = "1"
	if int(node) > 5 :
	    node = int(node) - 1
	shelf = int(node)/4
	node = int(node)%4
        position = "Shelf "+str(shelf)+ " Column "+str(node) +" Row "+pos
        code = [data , position]
        table.add_rows([code], header=False)

    #table.add_rows([codes, position], header=False)
    print table.draw()

def callback(data):
    string = data.data
    global qrdata
    if string in qrdata :
        return
    else :
        qrdata.append(string)
    handy(qrdata)

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/code/qr", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
