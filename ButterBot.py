import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from picarx import Picarx

# GLOBAL VARIABLES

CONFIDENCE_THRESHOLD = 0.1
yolov4 = True
if yolov4:
    LABELS_FILE = "weights/yolov4/coco.names"
    CONFIG_FILE = "weights/yolov4/yolov4_tiny.cfg"
    WEIGHTS_FILE = "weights/yolov4/yolov4-tiny_butter_best.weights"
else:
    LABELS_FILE = "weights/coco.names"
    CONFIG_FILE = "weights/yolov3.cfg"
    WEIGHTS_FILE = "weights/yolov3.weights"


def drawBoxes (image, layerOutputs, H, W, target, image_number):
  boxes = []
  confidences = []
  classIDs = []
  centerX_target = None
  for output in layerOutputs:
    for detection in output:
      scores = detection[5:]
      classID = np.argmax(scores)
      confidence = scores[classID]

      if confidence > CONFIDENCE_THRESHOLD:
        box = detection[0:4] * np.array([W, H, W, H])
        (centerX, centerY, width, height) = box.astype("int")

        x = int(centerX - (width / 2))
        y = int(centerY - (height / 2))

        print ("centerx: ", centerX)
        print ("centerY: ", centerY)
        print (LABELS[classID])
        
        if (LABELS[classID] == target):            
            centerX_target = centerX

        boxes.append([x, y, int(width), int(height)])
        confidences.append(float(confidence))
        classIDs.append(classID)

  idxs = cv2.dnn.NMSBoxes(boxes, confidences, CONFIDENCE_THRESHOLD, CONFIDENCE_THRESHOLD)

  # Ensure at least one detection exists
  if len(idxs) > 0:
    for i in idxs.flatten():
      (x, y) = (boxes[i][0], boxes[i][1])
      (w, h) = (boxes[i][2], boxes[i][3])

      color = [int(c) for c in COLORS[classIDs[i]]]

      cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
      text = "{}: {:.4f}".format(LABELS[classIDs[i]], confidences[i])
      cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

  # Display the image
  #cv2.imshow("output", image)
  cv2.imwrite("output/img_{}.png".format(image_number), image)
  #cv2.waitKey()
  return centerX_target

def get_search_angle(angle):
    if (angle == -35 or angle == 35):
        return 0
    
    if (angle < 0):
        return angle * -1 + 5
    else:
        return angle * -1 - 5   
  
def detectObjects (image, target, image_number):
    (H, W) = image.shape[:2]

    ln = net.getLayerNames()
    ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB = True, crop = False)
    net.setInput(blob)
    layerOutputs = net.forward(ln)
    
    return drawBoxes(image, layerOutputs, H, W, target, image_number)

def driveToTarget(px, swivel):
    # todo:
    # add distance measurement
    # by translating pixel data to distance sensor angle.
    # drive until this distance is small enough, than back up
    swivel = int(swivel)
    smoothing = 0.0001
    swivel_treshold = 5
    found = False
    dist = px.ultrasonic.read()
    print (dist)
    if dist < 5:
        return True
    
    print (swivel)
    if swivel < -swivel_treshold:
        for i in range(swivel, 0):
            px.set_dir_servo_angle(i)
            px.set_camera_servo1_angle(i)
            for i in range(0, 5):
                px.forward(4)
                time.sleep(0.05/abs(swivel*.75)+smoothing)
    if swivel > swivel_treshold :
        for i in range(swivel, 0, -1):
            px.set_dir_servo_angle(i)
            px.set_camera_servo1_angle(i)
            for i in range(0, 5):
                px.forward(4)
                time.sleep(0.05/(swivel*.75+smoothing))
    if -swivel_treshold < swivel < swivel_treshold:
        print("I am looking at our target")
        dist = px.ultrasonic.read()
        while dist > 5:
            px.forward(5)
            time.sleep(0.05)
            dist = px.ultrasonic.read()
        found = True

    
    px.forward(0)
    return found

def attachObject(px):
    for angle in range(-35, 35, 1):
        px.set_camera_servo1_angle(angle)
        time.sleep(0.1)
    return isAttached(px)

# Function to measure whether the object is attached to the robot
# We do this by check the distance to from the ultrasonic sensor
# by moving back a few cm and checking if the distance is still the same
def isAttached(px):
    curr_dist = px.ultrasonic.read()
    thresh = 1
    print (curr_dist)
    if (0 < curr_dist < 6):
        moveBack(px)
        new_dist = px.ultrasonic.read()
        print ("{} < {} < {}".format(curr_dist - thresh,new_dist,curr_dist + thresh))
        if (curr_dist - thresh < new_dist < curr_dist + thresh):
            print ()
            return True
    
    return False

def moveBack(px):
    for _ in range(0, 5):
        px.backward(5)
        time.sleep(0.05)
    px.forward(0)
    time.sleep(0.1)
    
            
def moveWheelDirection(px, swivel):
    px.set_dir_servo_angle(swivel)
    
def moveCamera(px, centerX):
    swivel = -35 + centerX / 9.1
    px.set_camera_servo1_angle(swivel)
    return swivel
            
def returnToOwner(px):
    for _ in range(0, 20):
        px.backward(5)
        time.sleep(0.1)
    px.forward(0)

def main():
    pass


if __name__ == '__main__':
    LABELS = open(LABELS_FILE).read().strip().split("\n")
    np.random.seed(4)
    COLORS = np.random.randint(0, 255, size = (len(LABELS), 3), dtype = "uint8")
    net = cv2.dnn.readNetFromDarknet(CONFIG_FILE, WEIGHTS_FILE)
    px = Picarx()
    px.set_camera_servo1_angle(0)
    px.set_dir_servo_angle(0)
    with PiCamera() as camera:
        camera.resolution = (640, 480)
        camera.framerate = 24
        rawCapture = PiRGBArray(camera, size=camera.resolution)
        
        time.sleep(2)
        search_angle = 0
        image_number = 0
        attached = False
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):  # use_video_port=True
            img = frame.array
            print ("loop")
            #start = time.time()
            #print ('start')
            target = "butter"
            centerX_target = detectObjects(img, target, image_number)
            image_number += 1
            dist = px.ultrasonic.read()
            print ("dist: ", dist)
            if centerX_target is not None:
                
                swivel = moveCamera(px, centerX_target)
                moveWheelDirection(px, swivel)
                found = driveToTarget(px, swivel)
                if (found):
                    attached = attachObject(px)
                    if (not attached):
                        moveBack(px)
                        moveBack(px)
                        moveBack(px)
                    else:
                        break
                time.sleep(0.5)
            else:
                search_angle = get_search_angle(search_angle)
                px.set_camera_servo1_angle(search_angle)
                time.sleep(0.5)
            print ("cup target: ", centerX_target)
            rawCapture.truncate(0)  # Release cache
        print ("uit forloop")
        moveBack(px)
        moveBack(px)
        print ("Klaar")
            


    


        