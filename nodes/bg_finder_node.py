#!/usr/bin/env python
from __future__ import print_function
import os
import rospy
import Queue
import threading
import yaml
import cv2
import numpy as np
import jpeg4py as jpeg

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge, CvBridgeError


class BgFinderNode(object):

    def __init__(self):


        rospy.init_node('bg_finder',log_level=rospy.INFO)

        self.use_compressed_images = rospy.get_param('/bg_finder/use_compressed_images',False)
        self.display_scale = rospy.get_param('/bg_finder/display_scale',1.0)
        self.update_dt = rospy.get_param('/bg_finder/update_dt', 2.0)
        self.max_len = rospy.get_param('/bg_finder/max_len', 50)
        default_output_file = os.path.join(os.path.abspath(os.curdir),'bg_image.npy')
        self.output_file = rospy.get_param('/bg_finder/output_file', default_output_file)

        self.bg_window = 'background'
        self.bg_last_update_time = 0.0

        self.bridge = CvBridge()
        self.img_queue = Queue.Queue()
        if self.use_compressed_images:
            self.img_sub = rospy.Subscriber('/camera/image_raw/compressed', CompressedImage, self.cmp_img_callback)
        else:
            self.img_sub = rospy.Subscriber('/camera/image_raw', Image, self.img_callback)

    def img_callback(self,ros_img): 
        cv_img = self.bridge.imgmsg_to_cv2(ros_img,desired_encoding='mono8')
        self.img_queue.put(cv_img)

    def cmp_img_callback(self,msg): 
        img_cmp = np.fromstring(msg.data, np.uint8)
        img_bgr = jpeg.JPEG(img_cmp).decode()
        img_gry = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        self.img_queue.put(img_gry)


    def run(self):

        bg_img = None

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
                if bg_img is None:
                    bg_img = new_img

                ros_time_now = rospy.Time.now()
                current_time = ros_time_now.to_time()
                elapsed_time = current_time - self.start_time 
                if elapsed_time - self.bg_last_update_time > self.update_dt:
                    bg_img = np.maximum(bg_img, new_img)
                    if self.use_compressed_images:
                        n,m = bg_img.shape
                        ns, ms = int(n*self.display_scale), int(m*self.display_scale)
                        bg_img_scaled = cv2.resize(bg_img,(ms,ns))
                        cv2.imshow(self.bg_window,bg_img_scaled)
                    else:
                        cv2.imshow(self.bg_window,bg_img)
                    cv2.moveWindow(self.bg_window, 200, 50)
                    cv2.waitKey(1)
                    self.bg_last_update_time = elapsed_time
                    

        rospy.logwarn('saving background image to {}'.format(self.output_file))
        with open(self.output_file,'w') as fid:
            np.save(self.output_file,bg_img)
        

if __name__ == '__main__':

    node = BgFinderNode()
    node.run()


