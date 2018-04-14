#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

import actionlib
import action_controller.msg
import ingress.msg
import copy
import math
import time

import numpy as np
from ingress_pomdp.vocab import Vocab

import pdb
import re

# Not used
CROP_X1 = 245
CROP_Y1 = 585
CROP_X2 = 245+1180
CROP_Y2 = 585+424

T = 15 # 17
V = 10510

densecap_vocab = Vocab('./ingress_pomdp/densecap_vocabulary.txt', T, V)
font = cv2.FONT_HERSHEY_SIMPLEX

def draw_boxes(img, boxes, idxs, ans_idx=0, belief=None, action=""):
    '''
    draws bboxes sorted by prob: [0] -> red, [1,2,3...] -> green
    '''
    # visualize
    draw_img = img.copy()
    # draw_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    for (count, idx) in enumerate(idxs):
        x1 = int(boxes[idx][0])
        y1 = int(boxes[idx][1])
        x2 = int(boxes[idx][0]+boxes[idx][2])
        y2 = int(boxes[idx][1]+boxes[idx][3])

        if count == ans_idx:
            cv2.rectangle(draw_img, (x1, y1), (x2, y2), (0,0,255), 12)
            if action != "":
                cv2.putText(draw_img, action, (x1 + int((x2-x1)/2.) - 20, int(y1 - 10)), font, 0.6, (0, 255, 0), 1, cv2.CV_AA)
            if belief != None:
                cv2.putText(draw_img, ('%.3f' % belief[count]), (x1 + int((x2-x1)/2.) - 20, int(y1 + 25)), font, 0.6, (0, 255, 0), 1, cv2.CV_AA)
        elif count < 5:
            cv2.rectangle(draw_img, (x1, y1), (x2, y2), (0,255,0), 2)
            if belief != None:
                cv2.putText(draw_img, ('%.3f' % belief[count]), (x1 + int((x2-x1)/2.) - 20, int(y1 + 25)), font, 0.6, (0, 255, 0), 1, cv2.CV_AA)

    return draw_img

            
def pub_image():

    print "Densecap - Vocabulary Size: %s" % (densecap_vocab.get_vocab_size())
    rospy.init_node('ImagePublisher', anonymous=True)

    client = actionlib.SimpleActionClient('dense_refexp_load', action_controller.msg.DenseRefexpLoadAction)
    print "Waiting for dense_refexp_load ..."
    client.wait_for_server()

    # Single Tabel Test
    path = './table_mult_dups.png'
    # path = './mult_obj_conf.png'

    img = cv2.imread(path,cv2.IMREAD_COLOR)
    img = cv2.resize(img,None,fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC)
    
    # img = img[CROP_Y1:CROP_Y2, CROP_X1:CROP_X2]
    # img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    msg_frame = CvBridge().cv2_to_imgmsg(img, "rgb8")
    goal = action_controller.msg.DenseRefexpLoadGoal(msg_frame)
    client.send_goal(goal)
    client.wait_for_result()  
    load_result = client.get_result()

    boxes = np.reshape(load_result.boxes, (-1, 4))
    semantic_captions = np.array(load_result.captions)
    semantic_losses = np.array(load_result.scores)
    semantic_word_probs = np.reshape(np.array(load_result.word_probs), (-1, T, V))
    semantic_top_prob_words = densecap_vocab.probs_to_captions(semantic_word_probs)
    # for c in semantic_top_prob_words:
    #     print c

    relevancy_client = actionlib.SimpleActionClient('relevancy_clustering', action_controller.msg.RelevancyClusteringAction)
    relevancy_client.wait_for_server()
    
    query_client = actionlib.SimpleActionClient('boxes_refexp_query', action_controller.msg.BoxesRefexpQueryAction)
    query_client.wait_for_server()

    planner_init_client = actionlib.SimpleActionClient('planner_init', ingress.msg.InitAction)
    planner_init_client.wait_for_server()

    planner_resp_client = actionlib.SimpleActionClient('planner_resp', ingress.msg.ResponseAction)
    planner_resp_client.wait_for_server()

    incorrect_idxs = []
    
    cv2.namedWindow('result', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('result', img.shape[1], img.shape[0])

    while True:
        # query = "the water bottle next to the green glass"
        query = raw_input('Search Query: ').lower()

        # relevancy clustering
        query_t0 = time.time()
        goal = action_controller.msg.RelevancyClusteringGoal(query, incorrect_idxs)
        relevancy_client.send_goal(goal)
        relevancy_client.wait_for_result()
        selected_orig_idx = relevancy_client.get_result().selection_orig_idx
        all_orig_idx = relevancy_client.get_result().all_orig_idx
        selected_boxes = np.take(boxes, selected_orig_idx, axis=0)
        semantic_softmax = np.array(relevancy_client.get_result().softmax_probs)
        semantic_softmax_orig_idxs = [-1.] * len(boxes)        
        for count, idx in enumerate(all_orig_idx):
            semantic_softmax_orig_idxs[idx] = semantic_softmax[count] 
                
        # refexp query
        goal = action_controller.msg.BoxesRefexpQueryGoal(query, selected_boxes.ravel(), selected_orig_idx,  incorrect_idxs)
        query_client.send_goal(goal)
        query_client.wait_for_result()
        query_result = query_client.get_result()
        query_t1 = time.time()
        print "Total Time for Query: " + str(query_t1-query_t0)
        
        # results
        top_idxs = [query_result.top_box_idx] + list(query_result.context_boxes_idxs)
        is_spatially_ambiguious = query_result.is_ambiguous
        spatial_captions = query_result.predicted_captions
        spatial_softmax = np.array(query_result.probs) / np.sum(np.array(query_result.probs))
        semantic_softmax = np.take(semantic_softmax, top_idxs)

        # pdb.set_trace()
        # send probs and captions to POMDP planner
        sem_captions = []
        sem_probs = []
        rel_captions = []
        rel_probs = []
        for (count, idx) in enumerate(top_idxs):
            sem_captions.append(semantic_captions[idx])
            sem_probs.append(semantic_softmax_orig_idxs[idx])
            print "Semantic - Prob(%f), Caption(%s)" % (semantic_softmax_orig_idxs[idx], semantic_captions[idx])
            if count < len(spatial_softmax) and count < len(spatial_captions):
                rel_captions.append(spatial_captions[count].replace(query, '').replace('.', ''))
                rel_probs.append(spatial_softmax[count])
                print "Spatial - Prob(%f), Caption(%s)" % (spatial_softmax[count], spatial_captions[count])
        goal = ingress.msg.InitGoal(query, sem_captions, sem_probs, rel_captions, rel_probs)
        planner_init_client.send_goal(goal)
        planner_init_client.wait_for_result()
        init_result = planner_init_client.get_result()
        init_belief = init_result.belief
        init_obj_idx = init_result.obj_idx
        init_action = init_result.action_type

        # disambiguation stage
        response = "NULL"
        action = "NULL"
        obj_idx = -1

        print init_result
        draw_img = draw_boxes(img, boxes, top_idxs, ans_idx=init_obj_idx, belief=init_belief, action=init_action)
        cv2.imshow('result', draw_img)
        k = cv2.waitKey(0)

        orig_top_idxs = list(top_idxs)        
        while True:
            if init_action == "PICK":
                break
            
            response = raw_input("Disamb: ").lower()
            if response == "!done":
                break
            
            goal = ingress.msg.ResponseGoal(response)
            planner_resp_client.send_goal(goal)
            planner_resp_client.wait_for_result()
            disamb_result = planner_resp_client.get_result()

            action = disamb_result.action_type
            obj_idx = disamb_result.obj_idx 
            belief = disamb_result.belief

            draw_img = draw_boxes(img, boxes, top_idxs, ans_idx=obj_idx, belief=belief, action=action)
            cv2.imshow('result', draw_img)
            k = cv2.waitKey(0)

            if action == "PICK":
                break
        
        # cv2.imwrite('./result_relevancy.png', draw_img)

    return True


if __name__ == '__main__':
    try:
        pub_image()
    except rospy.ROSInterruptException:
        pass
