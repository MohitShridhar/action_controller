#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from action_controller.cfg import RefexpMicoConfig

def callback(config, level):
    # rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\ 
    #       {str_param}, {bool_param}, {size}""".format(**config))
    print config
    return config

if __name__ == "__main__":
    rospy.init_node("refexp_mico_demo", anonymous = False)

    srv = Server(RefexpMicoConfig, callback)
    rospy.spin()
