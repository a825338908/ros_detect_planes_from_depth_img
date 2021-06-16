#!/usr/bin/env python
# -*- coding: utf-8 -*-

from ros_detect_planes_from_depth_img.msg import PlanesResults
from utils.lib_ros_rgbd_pub_and_sub import DepthImageSubscriber, ColorImageSubscriber, ColorImagePublisher, BiniaryImageSubscriber

import rospy

import numpy as np
from std_msgs.msg import Float32
import redis

red = redis.Redis(host="127.0.0.1", port=6379)

def main():
    threshold = 25
    #sub_depth = DepthImageSubscriber(args.depth_topic)
    color_topic = '/detect_plane/colored_mask'
    #args.color_topic
    sub_color = BiniaryImageSubscriber(color_topic)
    
    pub_spray_onoff = rospy.Publisher('spray_onoff', Float32, queue_size=1)

    # -- Wait for subscribing image and detect.
    onoff_val = 0.0
    onoff_prev = 0.0
    while not rospy.is_shutdown():
        if sub_color.has_image():
            color = sub_color.get_image()
            flat = color.flatten()
            zero_pct = np.count_nonzero(flat==0)*100.0/flat.size
            #print(zero_pct)
            if zero_pct < threshold:
                onoff_val = 1.0
            else:
                onoff_val = 0.0

            if red.get('main_switch_servo')!='1':
                onoff_val = 0.0
            print(onoff_val)
            if (onoff_val != onoff_prev):
                print(onoff_val, zero_pct)
                pub_spray_onoff.publish(onoff_val)
            onoff_prev = onoff_val
            # -- Publish result.
            #pub_colored_mask.publish(colored_mask)
            #pub_image_viz.publish(img_viz)
            #pub_results.publish(list_plane_params)
            #rospy.loginfo("Publish results completes.")
            #rospy.loginfo("-------------------------------------------------")
            #rospy.loginfo("")



if __name__ == '__main__':
    node_name = "kenson_spray"
    rospy.init_node(node_name)
    main()
    rospy.logwarn("Node `{}` stops.".format(node_name))