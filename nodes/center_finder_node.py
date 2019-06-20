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


class CenterFinderNode(object):

    def __init__(self):


        rospy.init_node('center_finder',log_level=rospy.INFO)

        self.threshold = rospy.get_param('/center_finder/threshold',15)
        self.min_area = rospy.get_param('/center_finder/min_area', 100)
        self.cmp_margin = rospy.get_param('/center_finder/cmp_margin', 5)
        self.filter_coeff = rospy.get_param('/center_finder/filter_coeff', 0.05)
        self.center_disp_radius = rospy.get_param('/center_finder/center_disp_radius', 3)
        default_output_file = os.path.join(os.path.abspath(os.curdir),'centers.yaml')
        self.output_file = rospy.get_param('/center_finder/output_file', default_output_file)

        self.centers_window = 'centers'
        self.threshold_window = 'threshold'
        self.is_first_img = True
        self.boundary_color = (0,255,0)
        self.center_color = (0,0,255)
        self.filtered_color = (0,255,255)

        self.centroid_array = None 
        self.filtered_centroid_array = None

        self.bridge = CvBridge()
        self.img_queue = Queue.Queue()
        self.img_sub = rospy.Subscriber('/camera/image_raw', Image, self.img_callback)


    def img_callback(self,ros_img): 
        cv_img = self.bridge.imgmsg_to_cv2(ros_img,desired_encoding='mono8')
        cv_img_bgr = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
        self.img_queue.put(cv_img_bgr)


    def run(self):

        while not rospy.is_shutdown():
            done = False
            new_img = None
            while not done:
                try:
                    new_img = self.img_queue.get_nowait()
                except Queue.Empty:
                    done = True
            if new_img is not None:
                centers_img, threshold_img, gray_img = self.find_centers_from_img(new_img)
                if self.is_first_img:
                    cv2.namedWindow(self.threshold_window,cv2.WINDOW_NORMAL)
                    cv2.moveWindow(self.threshold_window, 200, 250)
                    cv2.resizeWindow(self.threshold_window, 800,600)
                    cv2.namedWindow(self.centers_window,cv2.WINDOW_NORMAL)
                    cv2.moveWindow(self.centers_window, 220, 270)
                    cv2.resizeWindow(self.centers_window, 800,600)

                    self.is_first_img = False 
              
                cv2.imshow(self.centers_window,centers_img)
                cv2.imshow(self.threshold_window,threshold_img)
                cv2.waitKey(1)

        rospy.logwarn('saving center data to {}'.format(self.output_file))
        with open(self.output_file,'w') as fid:
            filtered_centroid_list = [[int(np.round(x)),int(np.round(y))] for x, y in self.filtered_centroid_array]
            yaml.dump(filtered_centroid_list, fid)


    def find_centers_from_img(self, image):
        image_gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        #image_gray = cv2.blur(image_gray,(3,3))
        rval, threshold_image = cv2.threshold(image_gray, self.threshold, np.iinfo(image.dtype).max, cv2.THRESH_BINARY)
        #threshold_image = cv2.morphologyEx(threshold_image, cv2.MORPH_CLOSE,np.ones((5,5))) 
        #threshold_image = cv2.dilate(threshold_image, np.ones((5,5)),50) 
        dummy, contour_list, dummy = cv2.findContours(threshold_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        filtered_contour_list = []
        centroid_list = []
        for contour in contour_list:
            moments = cv2.moments(contour) 
            if moments['m00'] > 0: 
                centroidX = int(np.round(moments['m10']/moments['m00']))
                centroidY = int(np.round(moments['m01']/moments['m00']))
            else:
                continue
            area = cv2.contourArea(contour)
            if area > self.min_area:
                filtered_contour_list.append(contour)
                centroid_list.append([int(centroidX),int(centroidY)])
                
        contours_image = cv2.cvtColor(image_gray,cv2.COLOR_GRAY2BGR)
        #cv2.drawContours(contours_image,filtered_contour_list,-1,self.boundary_color,1)


        centroid_list.sort(cmp=self.region_cmp_func)
        self.centroid_array = np.array(centroid_list)
        self.update_filtered_centroids()

        for cx,cy in self.centroid_array:
            cv2.circle(contours_image,(cx,cy),self.center_disp_radius,self.center_color)

        if self.filtered_centroid_array is not None:
            for cx,cy in self.filtered_centroid_array:
                cv2.circle(
                        contours_image,
                        (int(np.round(cx)),int(np.round(cy))),
                        self.center_disp_radius,
                        self.filtered_color
                        )
        return contours_image, threshold_image, image_gray 


    def update_filtered_centroids(self):

        if self.filtered_centroid_array is None:
            self.filtered_centroid_array = np.array(self.centroid_array)
        elif self.centroid_array.shape[0] != self.filtered_centroid_array.shape[0]:
            self.filtered_centroid_array = np.array(self.centroid_array)
        else:
            self.filtered_centroid_array  = (1-self.filter_coeff)*self.filtered_centroid_array 
            self.filtered_centroid_array += self.filter_coeff*self.centroid_array

    def region_cmp_func(self,p,q):
        """
        Comparison function for arena regions.  Sorts by x's first and the by y's in 
        inverse order.  Both x's and y's are considered equal if they are within the
        specified comparison margin (cmp_margin) of each other. 
        """
        px, py = p
        qx, qy = q
        rval = 0
        if px - qx > self.cmp_margin:
            rval = 1
        elif qx - px > self.cmp_margin:
            rval = -1
        if rval == 0:
            if py - qy > self.cmp_margin:
                rval = -1
            elif qy - py > self.cmp_margin:
                rval = 1
        return rval

        

if __name__ == '__main__':

    node = CenterFinderNode()
    node.run()


