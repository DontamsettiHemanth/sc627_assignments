#!/usr/bin/env python
import rospy
#from helper import *
if __name__ == "__main__":
    rospy.init_node("check")
    rospy.logwarn("Hit")#,True,0.512,str(point(1,1)))
    rospy.sleep(1)
    rospy.logerr("Hit")
    rospy.logwarn("Hit")
    exit(1)
    rospy.spin()
