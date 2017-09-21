#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

import actionlib
import action_controller.msg
import copy


import numpy as np

CROP_X1 = 245
CROP_Y1 = 585
CROP_X2 = 245+1180
CROP_Y2 = 585+424

def pub_image():
    rospy.init_node('ImagePublisher', anonymous=True)

    client = actionlib.SimpleActionClient('dense_refexp_load', action_controller.msg.DenseRefexpLoadAction)
    print "Waiting for dense_refexp_load ..."
    client.wait_for_server()

    # Single Tabel Test
    path = './table_mult_dups.png'

    img = cv2.imread(path,cv2.IMREAD_COLOR)
    img = img[CROP_Y1:CROP_Y2, CROP_X1:CROP_X2]
    # img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    msg_frame = CvBridge().cv2_to_imgmsg(img, "rgb8")
    goal = action_controller.msg.DenseRefexpLoadGoal(msg_frame)
    client.send_goal(goal)
    client.wait_for_result()  
    load_result = client.get_result()

    boxes = np.reshape(load_result.boxes, (-1, 4))      

    # Query test
    print "Waiting for relevancy_clustering server..."
    client = actionlib.SimpleActionClient('relevancy_clustering', action_controller.msg.RelevancyClusteringAction)
    client.wait_for_server()    
    print "Found relevancy_clustering server!"

    incorrect_idxs = []
    
    # cv2.namedWindow('result', cv2.WINDOW_NORMAL)
    # cv2.resizeWindow('result', img.shape[1], img.shape[0])

    while True:
        # query = "the water bottle next to the green glass"
        query = raw_input('Search Query: ').lower()

        goal = action_controller.msg.RelevancyClusteringGoal(query, incorrect_idxs)
        client.send_goal(goal)
        client.wait_for_result()
        selection_orig_idx = client.get_result().selection_orig_idx

        print selection_orig_idx
        
        # goal = action_controller.msg.DenseRefexpQueryGoal(query, incorrect_idxs)
        # client.send_goal(goal)
        # client.wait_for_result()
        # query_result = client.get_result()

        # top_idx = query_result.top_box_idx
        # context_boxes_idxs = list(query_result.context_boxes_idxs)
        # context_boxes_idxs.append(top_idx)

        # # visualize
        # draw_img = img.copy()
        # for (count, idx) in enumerate(context_boxes_idxs):

        #     x1 = int(boxes[idx][0])
        #     y1 = int(boxes[idx][1])
        #     x2 = int(boxes[idx][0]+boxes[idx][2])
        #     y2 = int(boxes[idx][1]+boxes[idx][3])

        #     if count == len(context_boxes_idxs)-1:
        #         cv2.rectangle(draw_img, (x1, y1), (x2, y2), (0,0,255), 12)
        #     else:
        #         cv2.rectangle(draw_img, (x1, y1), (x2, y2), (0,255,0), 2)

        # cv2.imshow('result', draw_img)
        # k = cv2.waitKey(0)

    return True


if __name__ == '__main__':
    try:
        pub_image()
    except rospy.ROSInterruptException:
        pass
