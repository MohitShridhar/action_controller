#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

import actionlib
import action_controller.msg
import copy
import math
import time

import numpy as np
from ingress_pomdp.vocab import Vocab

# Not used
CROP_X1 = 245
CROP_Y1 = 585
CROP_X2 = 245+1180
CROP_Y2 = 585+424

T = 17
V = 10510

densecap_vocab = Vocab('./ingress_pomdp/densecap_vocabulary.txt', T, V)


def pub_image():

    print "Densecap - Vocabulary Size: %s" % (densecap_vocab.get_vocab_size())
    rospy.init_node('ImagePublisher', anonymous=True)

    client = actionlib.SimpleActionClient('dense_localize_probs', action_controller.msg.LocalizeProbsAction)
    print "Waiting for dense_localize ..."
    client.wait_for_server()

    # Single Tabel Test
    path = './table_mult_dups.png'

    img = cv2.imread(path,cv2.IMREAD_COLOR)
    img = cv2.resize(img,None,fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
    
    # img = img[CROP_Y1:CROP_Y2, CROP_X1:CROP_X2]
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    msg_frame = CvBridge().cv2_to_imgmsg(img, "rgb8")
    goal = action_controller.msg.LocalizeGoal(1, msg_frame)
    client.send_goal(goal)
    client.wait_for_result()  
    load_result = client.get_result()

    boxes = np.reshape(load_result.boxes, (-1, 4))
    # print np.array(load_result.word_probs).shape, np.array(load_result.word_probs)[0]
    word_probs = np.reshape(load_result.word_probs, (-1, T, V))
    # print word_probs
    captions = densecap_vocab.probs_to_captions(word_probs)
    lua_losses = load_result.scores
    # print captions
    # print lua_losses
    
    query_client = actionlib.SimpleActionClient('localize_query', action_controller.msg.LocalizeQueryAction)
    query_client.wait_for_server()
    
    cv2.namedWindow('result', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('result', img.shape[1], img.shape[0])

    while True:
        # query = "the water bottle next to the green glass"
        query = raw_input('Search Query: ').lower()

        query_t0 = time.time()
        losses = densecap_vocab.simple_avg_cross_entropy_loss(word_probs, query)
        sorted_idx = np.argsort(losses)
        query_t1 = time.time()
        
        beam_length = len(load_result.captions)
        goal = action_controller.msg.LocalizeQueryGoal(query, beam_length, 6.0)
        query_client.send_goal(goal)
        query_client.wait_for_result()
        query_result = query_client.get_result()
        print "Total Time for Query: " + str(query_t1-query_t0)
        
        # orig_idx = sorted_idx
        orig_idx = query_result.orig_idx
        q_losses = query_result.captioning_losses
        # print q_losses
        
        # visualize
        draw_img = img.copy()
        draw_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        for (count, idx) in enumerate(orig_idx):

            x1 = int(boxes[idx][0])
            y1 = int(boxes[idx][1])
            x2 = int(boxes[idx][0]+boxes[idx][2])
            y2 = int(boxes[idx][1]+boxes[idx][3])

            if count == 0:
                cv2.rectangle(draw_img, (x1, y1), (x2, y2), (0,0,255), 12)
            elif count < 5:
                cv2.rectangle(draw_img, (x1, y1), (x2, y2), (0,255,0), 2)

        cv2.imshow('result', draw_img)
        k = cv2.waitKey(0)
        # cv2.imwrite('./result_relevancy.png', draw_img)

    return True


if __name__ == '__main__':
    try:
        pub_image()
    except rospy.ROSInterruptException:
        pass
