#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import UInt8
from math import radians 

w1 = radians(60)
vel_normal = 0.1
bump = 0
vel_flee = 0.26
def bumped(dado):
    global bump

    bump = dado.data
    print(bump)

if __name__ == "__main__":

    rospy.init_node("batida")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
    bumper_number = rospy.Subscriber("/bumper", UInt8, bumped)
    
    try:
        while not rospy.is_shutdown():
            backwards = Twist(Vector3(-vel_normal,0,0),Vector3(0,0,0))
            left = Twist(Vector3(0,0,0),Vector3(0,0,-w1))
            right = Twist(Vector3(0,0,0),Vector3(0,0,w1))
            stop = Twist(Vector3(0,0,0),Vector3(0,0,0))

            if bump == 1:
                print("I have bumped into something.")

                print("Going backwards...")
                pub.publish(backwards)
                rospy.sleep(1.5)

                print("I stopped.")
                pub.publish(stop)
                rospy.sleep(1.0)

                print("Avoiding object...")
                pub.publish(left)
                rospy.sleep(1.5)

                bump = None

            if bump == 2:
                print("I have bumped into something.")

                print("Going backwards...")
                pub.publish(backwards)
                rospy.sleep(1.5)

                print("I stopped.")
                pub.publish(stop)
                rospy.sleep(1.0)

                print("Avoiding object...")
                pub.publish(right)
                rospy.sleep(1.5)

                bump = None

            if bump == 3 or bump == 4:
                flee = Twist(Vector3(vel_flee,0,0),Vector3(0,0,0))
                print("Something bumped into me?")

                print("Running away!")
                pub.publish(flee)
                rospy.sleep(4.0)

                bump = None
                    
            else:
                print("Going normally.")
                go_lin = Twist(Vector3(vel_normal,0,0),Vector3(0,0,0))
                pub.publish(go_lin)
                rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("ROS.exe has stopped working.")