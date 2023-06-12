#!/usr/bin/env python
import rospy
from swarm_choosestation.msg import Publishpoint

def callback(data):
    rospy.loginfo(data.posx)

def func():
    print("Shutdown")

def listener():
    rospy.init_node('custom_listener', anonymous=True)
    rospy.Subscriber("publish_point", Publishpoint, callback)

    rospy.on_shutdown(func)
    rospy.signal_shutdown(func)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()