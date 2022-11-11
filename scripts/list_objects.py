#!/usr/bin/python

from utils import *
import rospy
import time

if __name__ == '__main__':
    try:

        l = list_object_names()
        print(l)

        obj = "pouch"
        p = get_object_pose(obj).position
        print("%s: %r" %(obj,p))

        obj = "robot"
        p = get_object_pose(obj).position
        print("%s: %r" %(obj,p))

    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
