#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import UInt8
from math import radians 

v = 1.0
w1 = radians(60)
w2 = radians(45)
vel_normal = 2.0
bump = 0

def bumped(data):
    global bump

    bump = data.data

if __name__ == "__main__":

    rospy.init_node("bumper")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)
    bumper_number = rospy.Subscriber("/bumper", UInt8, bumped)

    try:
        while not rospy.is_shutdown():
            backwards = Twist(Vector3(-v,0,0),Vector3(0,0,0))
            forwards = Twist(Vector3(v,0,0),Vector3(0,0,0))
            left = Twist(Vector3(0,0,0),Vector3(0,0,-w1))
            right = Twist(Vector3(0,0,0),Vector3(0,0,w1))
            stop = Twist(Vector3(0,0,0),Vector3(0,0,0))

            if bump == 1:
                print("Backwards")
                pub.publish(backwards)
                rospy.sleep(1.5)

                print("Stopped")
                pub.publish(stop)
                rospy.sleep(1.0)

                print("Turning right")
                pub.publish(right)
                rospy.sleep(1.5)

            elif bump == 2:
                print("Backwards")
                pub.publish(backwards)
                rospy.sleep(1.5)

                print("Stopped")
                pub.publish(stop)
                rospy.sleep(1.0)

                print("Turning left")
                pub.publish(left)
                rospy.sleep(1.5)

            elif bump == 3:
                print("Forwards")
                pub.publish(forwards)
                rospy.sleep(1.5)

                print("Stopped")
                pub.publish(stop)
                rospy.sleep(1.0)

                print("Turning right")
                pub.publish(right)
                rospy.sleep(1.5) 

            elif bump == 4:
                print("Forwards")
                pub.publish(forwards)
                rospy.sleep(1.5)

                print("Stopped")
                pub.publish(stop)
                rospy.sleep(1.0)

                print("Turning left")
                pub.publish(left)
                rospy.sleep(1.5)
                    
            else:
                print("Going")
                go_lin = Twist(Vector3(vel_normal,0,0),Vector3(0,0,0))
                pub.publish(go_lin)
                rospy.sleep(3.0)

                print("Stopped")
                pub.publish(stop)
                rospy.sleep(1.0)

                print("Turning")
                turn = Twist(Vector3(0,0,0),Vector3(0,0,w2))
                pub.publish(turn)
                rospy.sleep(1.0)
    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")