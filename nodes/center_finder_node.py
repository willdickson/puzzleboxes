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


class CenterFinderNode(object):

    def __init__(self):

        rospy.init_node('center_finder',log_level=rospy.INFO)

        self.use_compressed_images = rospy.get_param('/center_finder/use_compressed_images',False)
        self.display_scale = rospy.get_param('/center_finder/display_scale',1.0)
        self.threshold = rospy.get_param('/center_finder/threshold',15)
        self.min_area = rospy.get_param('/center_finder/min_area', 100)
        self.cmp_margin = rospy.get_param('/center_finder/cmp_margin', 5)
        self.close_kernel_size = rospy.get_param('/center_finder/close_kernel_size',0)
        self.erode_kernel_size = rospy.get_param('/center_finder/erode_kernel_size',0)
        self.filter_coeff = rospy.get_param('/center_finder/filter_coeff', 0.05)
        self.center_disp_radius = rospy.get_param('/center_finder/center_disp_radius', 3)
        self.center_disp_thickness = rospy.get_param('/center_finder/center_disp_thickness', 2)

        default_output_file = os.path.join(os.path.abspath(os.curdir),'centers.yaml')
        self.output_file = rospy.get_param('/center_finder/output_file', default_output_file)
        self.input_file = rospy.get_param('/center_finder/input_file', None)

        default_mask_npy_file = os.path.join(os.path.abspath(os.curdir),'mask.npy')
        self.mask_npy_file = rospy.get_param('/center_finder/mask_file', default_mask_npy_file)
        mask_npy_base_name, ext = os.path.splitext(self.mask_npy_file)
        self.mask_jpg_file = '{}.jpg'.format(mask_npy_base_name)

        self.centers_window = 'centers'
        self.threshold_window = 'threshold'
        self.is_first_img = True
        self.boundary_color = (0,255,0)
        self.center_color = (0,0,255)
        self.filtered_color = (0,255,255)

        self.centroid_array = None 
        self.filtered_centroid_array = None

        if self.input_file is not None:
            self.input_image = np.load(self.input_file)
        else:
            self.input_image = None

        self.bridge = CvBridge()
        self.img_queue = Queue.Queue()
        rospy.logwarn(self.use_compressed_images)
        if self.use_compressed_images:
            self.img_sub = rospy.Subscriber('/camera/image_raw/compressed', CompressedImage, self.cmp_img_callback)
        else:
            self.img_sub = rospy.Subscriber('/camera/image_raw', Image, self.img_callback)


    def img_callback(self,ros_img): 
        cv_img = self.bridge.imgmsg_to_cv2(ros_img,desired_encoding='mono8')
        if self.input_image is None:
            cv_img_bgr = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
        else:
            cv_img_bgr = cv2.cvtColor(self.input_image,cv2.COLOR_GRAY2BGR)
        self.img_queue.put(cv_img_bgr)

    def cmp_img_callback(self,msg): 
        if self.input_image is None:
            img_cmp = np.fromstring(msg.data, np.uint8)
            img_bgr = jpeg.JPEG(img_cmp).decode()
        else:
            img_bgr = cv2.cvtColor(self.input_image,cv2.COLOR_GRAY2BGR)
        self.img_queue.put(img_bgr)


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
                centers_img, centers_img_scaled, threshold_img, threshold_img_scaled, gray_img = self.find_centers_from_img(new_img)
                if self.is_first_img:
                    cv2.namedWindow(self.threshold_window,cv2.WINDOW_NORMAL)
                    cv2.moveWindow(self.threshold_window, 200, 250)
                    cv2.resizeWindow(self.threshold_window, 800,600)
                    cv2.namedWindow(self.centers_window,cv2.WINDOW_NORMAL)
                    cv2.moveWindow(self.centers_window, 220, 270)
                    cv2.resizeWindow(self.centers_window, 800,600)
                    self.is_first_img = False 

                cv2.imshow(self.centers_window,centers_img_scaled)
                cv2.imshow(self.threshold_window,threshold_img_scaled)
                cv2.waitKey(1)

        rospy.logwarn('saving center data to {}'.format(self.output_file))
        with open(self.output_file,'w') as f:
            filtered_centroid_list = [[int(np.round(x)),int(np.round(y))] for x, y in self.filtered_centroid_array]
            yaml.dump(filtered_centroid_list, f)

        np.save(self.mask_npy_file, threshold_img)
        cv2.imwrite(self.mask_jpg_file, threshold_img)


    def find_centers_from_img(self, image):
        image_gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        #image_gray = cv2.blur(image_gray,(3,3))
        rval, threshold_image = cv2.threshold(image_gray, self.threshold, np.iinfo(image.dtype).max, cv2.THRESH_BINARY)
        if self.close_kernel_size > 0:
            close_kernel = np.ones((self.close_kernel_size, self.close_kernel_size))
            threshold_image = cv2.morphologyEx(threshold_image, cv2.MORPH_CLOSE, close_kernel) 
        if self.erode_kernel_size > 0:
            erode_kernel = np.ones((self.erode_kernel_size, self.erode_kernel_size))
            threshold_image = cv2.erode(threshold_image, erode_kernel, 1) 
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

        if self.use_compressed_images: 
            scale = self.display_scale
            n,m,c = contours_image.shape
            ns, ms = int(n*scale), int(m*scale)
            contours_image_scaled = cv2.resize(contours_image,(ms,ns),0, 0, cv2.INTER_AREA)
            threshold_image_scaled = cv2.resize(threshold_image,(ms,ns),0, 0, cv2.INTER_AREA)
        else:
            scale = 1.0

        for cx,cy in self.centroid_array:
            cv2.circle(contours_image_scaled,(int(scale*cx),int(scale*cy)),self.center_disp_radius,self.center_color,self.center_disp_thickness)

        # Need to scale contours
        # -------------------------------------------------------------------------------------
        #if self.filtered_centroid_array is not None:
        #    for cx,cy in self.filtered_centroid_array:
        #        cv2.circle(
        #                contours_image_scaled,
        #                (int(np.round(scale*cx)),int(np.round(scale*cy))),
        #                self.center_disp_radius,
        #                self.filtered_color
        #                )
        # -------------------------------------------------------------------------------------
        return contours_image, contours_image_scaled, threshold_image, threshold_image_scaled, image_gray 


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


