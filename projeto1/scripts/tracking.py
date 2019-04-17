from imutils.video import VideoStream, FPS
import imutils
import time
import numpy as np
import cv2

detect_args = ["MobileNetSSD_deploy.prototxt.txt","MobileNetSSD_deploy.caffemodel",0.2]

CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
            "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
            "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
            "sofa", "train", "tvmonitor"]

print("="*9+" CLASSES "+"="*9)
print("")
for i in range(1,len(CLASSES)):
    print(f"{i} : {CLASSES[i-1]}")
print("")        
print("="*27)

objeto = int(input("Choose a class: "))

COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

print("[INFO] loading model...")
net = cv2.dnn.readNetFromCaffe(detect_args[0], detect_args[1])

def detect(frame):

    image = frame.copy()
    (h, w) = image.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)

    print("[INFO] computing object detections...")
    net.setInput(blob)
    detections = net.forward()

    results = []

    for i in np.arange(0, detections.shape[2]):

        confidence = detections[0, 0, i, 2]

        if confidence > detect_args[2]:

            idx = int(detections[0, 0, i, 1])
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")

            if CLASSES[idx] == objeto:

                label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
                print("[INFO] {}".format(label))
                cv2.rectangle(image, (startX, startY), (endX, endY),
                    COLORS[idx], 2)
                y = startY - 15 if startY - 15 > 15 else startY + 15
                cv2.putText(image, label, (startX, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

                results.append((CLASSES[idx], confidence*100, (startX, startY),(endX, endY) ))

    return image, results

OPENCV_OBJECT_TRACKERS = {1:{"csrt": cv2.TrackerCSRT_create},
                          2:{"kcf": cv2.TrackerKCF_create},
                          3:{"boosting": cv2.TrackerBoosting_create},
                          4:{"mil": cv2.TrackerMIL_create},
                          5:{"tld": cv2.TrackerTLD_create},
                          6:{"medianflow": cv2.TrackerMedianFlow_create},
                          7:{"mosse": cv2.TrackerMOSSE_create}}

print("="*5+"TYPES OF TRACKING"+"="*5)
print("")

for t in OPENCV_OBJECT_TRACKERS:
    for k, v in OPENCV_OBJECT_TRACKERS[i].items():
        print(f"{t} - {k}")

print("")        
print("="*27)

track_type = int(input("Choose your tracking type: "))

for name, function in OPENCV_OBJECT_TRACKERS[track_type].items():
    tracker_name = name
    tracker = function()

initBB = None

fps = None

cap = cv2.VideoCapture(0)

qtd_frames = 0

detecting = True

while(True):
    ret, frame = cap.read()
    (H, W) = frame.shape[:2]

    if detecting:
        result_frame, result_tuples = detect(frame)
        
        for tup in result_tuples:
            if tup[0] == objeto:
                qtd_frames += 1
            else:
                qtd_frames = 0

        if len(result_tuples) == 0:
            qtd_frames = 0
            
        if qtd_frames >= 5:
            detecting = False
            cv2.destroyAllWindows()

        cv2.imshow('Detected Objects', result_frame)

    else:        
        if initBB == None:
            pos_x1, pos_y1 = result_tuples[0][2]
            pos_x2, pos_y2 = result_tuples[0][3]
            width = abs(pos_x2 - pos_x1)
            height = abs(pos_y2 - pos_y1)

            initBB = (pos_x1, pos_y1, width,height)

            tracker.init(frame, initBB)
            fps = FPS().start()

        if initBB is not None:
            
            (success, box) = tracker.update(frame)

            if success:
                (x, y, w, h) = [int(v) for v in box]
                cv2.rectangle(frame, (x, y), (x + w, y + h),
                    (0, 255, 0), 2)

            else:
                initBB = None
                qtd_frames = 0

            fps.update()
            fps.stop()

            info = [
                ("Tracker", tracker_name),
                ("Success", "Yes" if success else "No"),
                ("FPS", "{:.2f}".format(fps.fps())),
            ]

            for (i, (k, v)) in enumerate(info):
                text = "{}: {}".format(k, v)
                cv2.putText(frame, text, (10, H - ((i * 20) + 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.imshow("Frame", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()