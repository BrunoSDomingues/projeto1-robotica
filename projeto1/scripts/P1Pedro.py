#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import tf
import math
import cv2
import time
from std_msgs.msg import UInt8
from math import radians 
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cormodule

def scaneou(dado):
    global dist

	lista = np.array(dado.ranges).round(decimals=2)
	dist = lista[-1]

w1 = radians(60)
vel_normal = 0.1
bump = 0
vel_flee = 0.26

def bumped(dado):
    global bump

    bump = dado.data
    print(bump)

bridge = CvBridge()
cv_image = None
media = []
centro = []
atraso = 1.5E9 
area = 0.0 
check_delay = False

def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime 
	delay = lag.nsecs
	print("delay ", "{:.3f}".format(delay/1.0E9))
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media, centro, area =  cormodule.identifica_cor(cv_image)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)

if __name__=="__main__":
	rospy.init_node("all")

	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
    bumper_number = rospy.Subscriber("/bumper", UInt8, bumped)

	topico_imagem = "/kamera" 
	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
	print("Usando ", topico_imagem)

	try:

		while not rospy.is_shutdown():
            # From LaserScan
		    velocidade = Twist(Vector3(v, 0, 0), Vector3(0, 0, 0))
            pub.publish(velocidade)
		    rospy.sleep(2)

            # From Bump 
            backwards = Twist(Vector3(-vel_normal,0,0),Vector3(0,0,0))
            left = Twist(Vector3(0,0,0),Vector3(0,0,-w1))
            right = Twist(Vector3(0,0,0),Vector3(0,0,w1))
            stop = Twist(Vector3(0,0,0),Vector3(0,0,0))

            if dist < 0.5:
                w = 0.5
                turnback = Twist(Vector3(-vel_normal, 0, 0), Vector3(0, 0, w))
                pub.publish(turnback)
                rospy.sleep(2)

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
                    
            # From Cor
			if len(media) != 0 and len(centro) != 0:
				print("Média dos vermelhos: {0}, {1}".format(media[0], media[1]))
				print("Centro dos vermelhos: {0}, {1}".format(centro[0], centro[1]))
				vel = Twist(Vector3(0.05,0,0), Vector3(0,0,0))
                pub.publish(vel)
				rospy.sleep(0.1)

				if centro[0] - media[0] < 10:
					print(centro[0] - media[0])
					vel = Twist(Vector3(0.5,0,0), Vector3(0,0,1.0))
                    pub.publish(vel)
					rospy.sleep(0.1)

				elif centro[0] - media[0] > 10:
					print(centro[0] - media[0])
					vel = Twist(Vector3(0.5,0,0), Vector3(0,0,-1.0))
                    pub.publish(vel)
					rospy.sleep(0.1)

            else:
                print("Going normally.")
                go_lin = Twist(Vector3(vel_normal,0,0),Vector3(0,0,0))
                pub.publish(go_lin)
                rospy.sleep(0.1)

		
	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")