#! /usr/bin/env python
# -*- coding:utf-8 -*-

# Imports #

# Modules

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from math import radians, pi
from std_msgs.msg import UInt8
import numpy as np
import cv2

# Extra functions

from mobilenet_object import detecta as det

# Variables #

# Globals

dist = 0
bridge = CvBridge()
cv_image = None
reslist = []
cen = 150

# Speeds

w1 = radians(60)
vel_normal = 0.1
vel_flee = 0.26

flee = Twist(Vector3(vel_flee,0,0),Vector3(0,0,0))
slow = Twist(Vector3(round(-vel_normal/3),0,0),Vector3(0,0,0))
turn = Twist(Vector3(0,0,0),(0,0,round(pi,4)))

forwards = Twist(Vector3(vel_normal,0,0),Vector3(0,0,0))
backwards = Twist(Vector3(-vel_normal,0,0),Vector3(0,0,0))
left = Twist(Vector3(0,0,0),Vector3(0,0,-w1))
right = Twist(Vector3(0,0,0),Vector3(0,0,w1))
stop = Twist(Vector3(0,0,0),Vector3(0,0,0))

# Functions #

def scaneou(dado):
    global dist
    
    lista = np.array(dado.ranges).round(decimals=2)
    dist = lista[-1]

def everytime(frame):
    global cv_image
    global reslist, cen

    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(frame, "bgr8")
        cv_image, reslist, cen = det(cv_image)
        cv2.imshow("Camera", cv_image)

    except CvBridgeError as e:
        print("An error has occurred", e)


# Main loop #

if __name__ == "__main__":

    rospy.init_node("AllInOne")

    topico_imagem = "/kamera"
    recebe = rospy.Subscriber(topico_imagem, CompressedImage, everytime, queue_size=10, buff_size=2**24)
    print("Usando o topico "+topico_imagem)

    scan = rospy.Subscriber("/scan", LaserScan, scaneou)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    try:
        while not rospy.is_shutdown():

            print("Going normally...")
            pub.publish(forwards)
            rospy.sleep(1.5)

            if dist < 0.5:
                print("There is something in front of me!")
                pub.publish(stop)
                w = 0.5
                turnback = Twist(Vector3(-vel_normal, 0, 0), Vector3(0, 0, w))
                pub.publish(turnback)
                rospy.sleep(3.0)

            else:
                if len(reslist) != 0:
                    for res in reslist:
                        if res[0] == "car":
                            print("A car! I'll back up slowly...")
                            pub.publish(slow)
                            rospy.sleep(1.5)
                            print("Turn...")
                            pub.publish(turn)
                            rospy.sleep(1.0)
                            print("And run away!")
                            pub.publish(flee)
                            rospy.sleep(1.5)

                        elif res[0] == "bird":
                            cen = int((res[3][0]+res[2][0])/2)
                            print(cen)
                            print("A bird! I'll follow it!")
                            if cen > 340:
                                print("I need to turn!")
                                pub.publish(left)
                                rospy.sleep(0.5)
                            elif cen < 300:
                                print("I need to turn!")
                                pub.publish(right)
                                rospy.sleep(0.5)


    except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")