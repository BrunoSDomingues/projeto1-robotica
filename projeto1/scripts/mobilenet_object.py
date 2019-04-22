#! /usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np
import cv2

detect_args = ["MobileNetSSD_deploy.prototxt.txt","MobileNetSSD_deploy.caffemodel",0.2]

CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
            "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
            "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
            "sofa", "train", "tvmonitor"]

COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))
centrox = 320

# detecta

def detecta(frame):
    global centrox 

    img = frame.copy()
    h, w = img.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(img, (300, 300)), 0.007843, (300, 300), 127.5)
    # print("[INFO] computing object detections...")
    net = cv2.dnn.readNetFromCaffe(detect_args[0], detect_args[1])
    net.setInput(blob)
    detections = net.forward()

    results = []

    for i in np.arange(0, detections.shape[2]):

        confidence = detections[0, 0, i, 2]

        if confidence > detect_args[2]:

            idx = int(detections[0, 0, i, 1])
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")

            if CLASSES[idx] == "car" or CLASSES[idx] == "bird":
                label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
                print("[INFO] {}".format(label))
                cv2.rectangle(img, (startX, startY), (endX, endY), COLORS[idx], 2)
                y = startY - 15 if startY - 15 > 15 else startY + 15
                cv2.putText(img, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

                results.append((CLASSES[idx], confidence*100, (startX, startY),(endX, endY)))

                centrox = round((endX+startX)/2)

    return img, results, centrox