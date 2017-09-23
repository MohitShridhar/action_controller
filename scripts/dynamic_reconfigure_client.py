#!/usr/bin/env python

import rospy

import dynamic_reconfigure.client

def callback(config):
    print config
    
if __name__ == "__main__":
    rospy.init_node("refexp_mico_client")

    client = dynamic_reconfigure.client.Client("refexp_mico_demo", timeout=30, config_callback=callback)
    rospy.spinOnce()
