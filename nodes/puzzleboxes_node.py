#!/usr/bin/env python
from __future__ import print_function
import os
import Queue
import roslib
import rospy
import numpy
import std_msgs.msg
import cv2
import threading
import yaml
import time
import pprint
pp = pprint.PrettyPrinter(indent=4)


from tracking_region import TrackingRegion
from region_visualizer import RegionVisualizer


from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from multi_tracker.msg import Trackedobject, Trackedobjectlist

class PuzzleBoxes(object):

    Default_Param_File = 'puzzleboxes_param.yaml'

    def __init__(self,nodenum=1):

        self.objects_queue = Queue.Queue()

        rospy.init_node('puzzleboxes')
        self.start_time = rospy.Time.now().to_time()

        # Read parameters
        self.param_path = '/puzzleboxes'
        self.get_param()
        self.check_param()

        self.create_tracking_regions()
        self.region_visualizer = RegionVisualizer(self.tracking_region_list)

        # Subscribe to tracked objects topic
        tracked_objects_topic = '/multi_tracker/{}/tracked_objects'.format(nodenum)
        self.tracked_objects_sub = rospy.Subscriber(
                tracked_objects_topic, 
                Trackedobjectlist, 
                self.tracked_objects_callback
                )

        # Subscribe to camera images
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_lock = threading.Lock()
        #self.image_sub = rospy.Subscriber('/camera/image_mono', Image, self.image_callback)
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        #self.data_pub = rospy.Publisher('puzzleboxes_data', PuzzleBoxesData, queue_size=10) 

    def get_param(self):
        self.param = rospy.get_param(self.param_path, None)
        if self.param is None:
            param_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),self.Default_Param_File)
            with open(param_file_path,'r') as f:
                self.param = yaml.load(f)

    def check_param(self):
        assert len(self.param['regions']['centers']) == len(self.param['regions']['protocols'])


    def create_tracking_regions(self):
        self.tracking_region_list = []
        center_list =  self.param['regions']['centers']
        protocol_list =  self.param['regions']['protocols']
        for region_index, region_data in enumerate(zip(center_list, protocol_list)):
            region_center_pt, region_protocol = region_data
            cx, cy = region_center_pt
            # Get lower left and upper right corners of the tracking region
            x0 = int(cx - 0.5*self.param['regions']['width'])
            y0 = int(cy - 0.5*self.param['regions']['height'])
            x1 = int(cx + 0.5*self.param['regions']['width'])
            y1 = int(cy + 0.5*self.param['regions']['height'])
            region_param = {
                    'index'         :  region_index,
                    'protocol'      :  region_protocol,
                    'center'        :  {'cx': cx, 'cy': cy, }, 
                    'roi'           :  {'x0': x0, 'x1': x1, 'y0': y0, 'y1': y1}, 
                    'default_param' :  self.param['default_param'],
            }
            self.tracking_region_list.append(TrackingRegion(region_param))

    def image_callback(self,ros_img): 
        cv_img = self.bridge.imgmsg_to_cv2(ros_img,desired_encoding='mono8')
        cv_img_bgr = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
        with self.image_lock:
            self.latest_image = cv_img_bgr

    def tracked_objects_callback(self,data):
        number_of_objects = len(data.tracked_objects)
        if number_of_objects > 0: 
            self.objects_queue.put(data.tracked_objects)

    def run(self):

        while not rospy.is_shutdown():
            while (self.objects_queue.qsize() > 0):
                # Process tracked objects
                tracked_objects = self.objects_queue.get()
                self.process_regions(tracked_objects)

            # Visualize regions and objecs
            with self.image_lock:
                self.region_visualizer.update(self.latest_image, [])

    def process_regions(self,tracked_objects):

        ros_time_now = rospy.Time.now()
        current_time = ros_time_now.to_time()
        elapsed_time = current_time - self.start_time 

#        header = std_msgs.msg.Header()
#        header.stamp = ros_time_now
#
#        msg = PathIntegration3x3Data()
#        msg.header = header

        for tracking_region in self.tracking_region_list: 
            tracking_region.update(elapsed_time, tracked_objects)
#            msg.tracking_region_data.append(region_data)
#        print()

#
#        self.data_pub.publish(msg)
                

# Utility functions
# -------------------------------------------------------------------------------------------------------

#def convert_region_data_list(region_data_list):
#    return [convert_region_data(r) for r in region_data_list]
#
#def convert_region_data(raw_region_data):
#    region_data = dict(raw_region_data)
#    region_data['leds'] = convert_led_list(region_data['leds'])
#    return region_data
#
#def convert_led_list(led_data_list):
#    return [convert_led_data(d) for d in led_data_list]
#
#def convert_led_data(led_data): 
#    x = led_data['x']
#    y = led_data['y']
#    w = led_data['w']
#    h = led_data['h']
#    x0 = x - w/2
#    y0 = y - h/2
#    x1 = x + w/2
#    y1 = y + h/2
#    return {'x0':x0, 'y0':y0, 'x1':x1, 'y1':y1, 'chan': led_data['chan']}



# -------------------------------------------------------------------------------------------------------
if __name__ == '__main__':

    node = PuzzleBoxes()
    node.run()
