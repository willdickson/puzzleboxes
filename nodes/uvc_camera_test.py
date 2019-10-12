#!/usr/bin/env python
from __future__ import print_function
import rospy
import cv2
import numpy as np
import time
import jpeg4py as jpeg
import Queue

from sensor_msgs.msg import CompressedImage

class CameraTest(object):

    def __init__(self):
        rospy.init_node('cameratest')
        self.rate = rospy.Rate(10)

        self.image_sub = rospy.Subscriber('/camera/image_raw/compressed',CompressedImage, self.image_callback)

        self.t_last = time.time()
        self.queue = Queue.Queue()

    def image_callback(self,msg):
        t_now = time.time()
        dt = t_now - self.t_last
        self.t_last = t_now
        img_cmp = np.fromstring(msg.data, np.uint8)
        img_bgr = jpeg.JPEG(img_cmp).decode()
        img_gry = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        self.queue.put(img_gry)

        print('image_callback: {}, hz = {}'.format(img_gry.shape,1.0/dt))


    def run(self):
        while not rospy.is_shutdown():
            img = None
            while True:
                try:
                    img = self.queue.get_nowait()
                    #print('*', end='')
                except Queue.Empty:
                    break

            if img is not None:

                if 0:
                    #print()
                    s = 0.4 
                    #n,m,c = img.shape
                    n,m = img.shape
                    ns, ms = int(n*s), int(m*s)
                    img_ds = cv2.resize(img,(ms,ns))
                    cv2.imshow('image',img_ds)
                else:
                    cv2.imshow('image',img)
                cv2.waitKey(1)
            #self.rate.sleep()






# -----------------------------------------------------------------------------------
if __name__ == '__main__':

    node = CameraTest()
    node.run()

