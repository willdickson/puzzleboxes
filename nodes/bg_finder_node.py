#!/usr/bin/env python
from __future__ import print_function
import os
import rospy
import Queue
import threading
import yaml
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class BgFinderNode(object):

    def __init__(self):

        rospy.init_node('bg_finder',log_level=rospy.INFO)

        self.update_dt = rospy.get_param('/bg_finder/update_dt', 2.0)
        self.max_len = rospy.get_param('/bg_finder/max_len', 50)
        default_output_file = os.path.join(os.path.abspath(os.curdir),'bg_image.npy')
        self.output_file = rospy.get_param('/bg_finder/output_file', default_output_file)

        self.bg_list = []
        self.bg_window = 'background'
        self.bg_last_update_time = 0.0

        self.bridge = CvBridge()
        self.img_queue = Queue.Queue()
        self.img_sub = rospy.Subscriber('/camera/image_raw', Image, self.img_callback)


    def img_callback(self,ros_img): 
        cv_img = self.bridge.imgmsg_to_cv2(ros_img,desired_encoding='mono8')
        self.img_queue.put(cv_img)


    def run(self):

        self.start_time = rospy.Time.now().to_time()

        while not rospy.is_shutdown():
            done = False
            new_img = None
            while not done:
                try:
                    new_img = self.img_queue.get_nowait()
                except Queue.Empty:
                    done = True
            if new_img is not None:
                ros_time_now = rospy.Time.now()
                current_time = ros_time_now.to_time()
                elapsed_time = current_time - self.start_time 
                if not self.bg_list or elapsed_time - self.bg_last_update_time > self.update_dt: 
                    print(len(self.bg_list))
                    self.bg_list.append(new_img)
                    self.bg_last_update_time = elapsed_time
                    if len(self.bg_list) > self.max_len:
                        self.bg_list.pop(0)
                    if len(self.bg_list) > 2:
                        #bg_img = np.median(np.array(self.bg_list), axis=0)
                        bg_img = np.max(np.array(self.bg_list), axis=0)
                        bg_img = bg_img.astype(np.uint8)
                    else:
                        bg_img = self.bg_list[0]
                    cv2.imshow(self.bg_window,bg_img)
                    cv2.moveWindow(self.bg_window, 200, 250)
                    cv2.waitKey(1)

        rospy.logwarn('saving background image to {}'.format(self.output_file))
        with open(self.output_file,'w') as fid:
            np.save(self.output_file,bg_img)
        

if __name__ == '__main__':

    node = BgFinderNode()
    node.run()


