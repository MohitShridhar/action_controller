#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

import actionlib
import action_controller.msg
import copy

imgs = []
result_img = None

build_map = True

IMAGE_SIZE = 720
NUM_PROPOSALS = 50
NUM_BOXES = 20
DENSITY = 0.7
OVERLAP = 0.3
SAVE_HISTORY = True

def pub_image():
    rospy.init_node('ImagePublisher', anonymous=True)

    client = actionlib.SimpleActionClient('dense_caption', action_controller.msg.DenseCaptionAction)
    client.wait_for_server()

    # Load Images (x3). Extracts features and store them in history. 
    img = cv2.imread('zebra.jpg',cv2.IMREAD_COLOR)
    imgs.append(img)
    msg_frame = CvBridge().cv2_to_imgmsg(img, "bgr8")
    if build_map:
        goal = action_controller.msg.DenseCaptionGoal(1, msg_frame, IMAGE_SIZE, NUM_PROPOSALS, NUM_BOXES, DENSITY, OVERLAP, SAVE_HISTORY)
        client.send_goal(goal)
        client.wait_for_result()

    img = cv2.imread('terman_eng.jpg',cv2.IMREAD_COLOR)
    imgs.append(img)
    msg_frame = CvBridge().cv2_to_imgmsg(img, "bgr8")
    if build_map:
        goal = action_controller.msg.DenseCaptionGoal(2, msg_frame, IMAGE_SIZE, NUM_PROPOSALS, NUM_BOXES, DENSITY, OVERLAP, SAVE_HISTORY)
        client.send_goal(goal)
        client.wait_for_result()

    img = cv2.imread('living_room.png',cv2.IMREAD_COLOR)
    imgs.append(img)
    msg_frame = CvBridge().cv2_to_imgmsg(img, "bgr8")
    if build_map:
        goal = action_controller.msg.DenseCaptionGoal(3, msg_frame, IMAGE_SIZE, NUM_PROPOSALS, NUM_BOXES, DENSITY, OVERLAP, SAVE_HISTORY)
        client.send_goal(goal)
        client.wait_for_result()



    # Query with a description
    query = "the wooden chair"
    min_loss_threshold = 0.1
    client = actionlib.SimpleActionClient('dense_query', action_controller.msg.DenseImageQueryAction)
    client.wait_for_server()    
    goal = action_controller.msg.DenseImageQueryGoal(query, min_loss_threshold)
    client.send_goal(goal)
    client.wait_for_result()

    result = client.get_result()


    # Plot results on image
    for i in range(0, 3):
        img = imgs[result.frame_ids[i]-1]
        offset = i*4

        x1 = int(result.boxes[0+offset])
        y1 = int(result.boxes[1+offset])
        x2 = int(result.boxes[0+offset]+result.boxes[2+offset])
        y2 = int(result.boxes[1+offset]+result.boxes[3+offset])

        cv2.namedWindow('result', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('result', 640, 480)
        cv2.rectangle(img, (x1, y1), (x2, y2), (255,0,0), 2)

        cv2.imshow('result', img)
        k = cv2.waitKey(0)

    # return client.get_result()  
    

if __name__ == '__main__':
    try:
        pub_image()
    except rospy.ROSInterruptException:
        pass