#!/usr/bin/env python
from __future__ import print_function
import os
import Queue
import roslib
import rospy
import std_msgs.msg
import cv2
import threading
import yaml
import json
import time
import datetime
import pandas as pd
import numpy as np

from phidgets1031_led_controller import LedController

from tracking_region import TrackingRegion
from region_visualizer import RegionVisualizer
from trial_scheduler import TrialScheduler

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from multi_tracker.msg import Trackedobject, Trackedobjectlist
from puzzleboxes.msg import PuzzleboxesData
from puzzleboxes.msg import RegionData

class PuzzleBoxes(object):

    Default_Param_File = 'puzzleboxes_param.yaml'

    def __init__(self,nodenum=1):

        self.devices = {}
        self.devices['led_controller'] = LedController()
        self.objects_queue = Queue.Queue()

        rospy.init_node('puzzleboxes')

        # Read parameters
        self.param_path = '/puzzleboxes'
        self.get_param()
        self.check_param()

        self.create_tracking_regions()
        self.region_visualizer = RegionVisualizer(self.tracking_region_list)
        self.trial_scheduler = TrialScheduler(self.param['trial_schedule'])

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
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.data_pub = rospy.Publisher('/puzzleboxes_data', PuzzleboxesData, queue_size=10) 
        self.param_pub = rospy.Publisher('/puzzleboxes_param', std_msgs.msg.String,queue_size=10)
        self.param_pub_timer = rospy.Timer(rospy.Duration(secs=self.param['param_pub_period']), self.on_param_pub_timer)

    def on_param_pub_timer(self,event):
        self.param_pub.publish(json.dumps(self.param))

    def get_param(self):
        self.param = rospy.get_param(self.param_path, None)
        if self.param is None:
            param_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),self.Default_Param_File)
            rospy.logwarn('puzzleboxes param not on server ... loading file')
            rospy.logwarn('param file = {}'.format(param_file_path))
            with open(param_file_path,'r') as f:
                self.param = yaml.load(f)
        self.load_trial_param_from_csv()
        now = datetime.datetime.now()
        self.param['datetime']= now.strftime('%m%d%y_%H%M%S')

    def load_trial_param_from_csv(self):
        df = pd.read_csv(self.param['trial_param_file'])

        # Load protocol paramters
        protocol_list = []
        for i in range(len(self.param['regions']['centers'])):
            protocol = {}
            if type(df['Fly'][i]) == str:
                protocol['fly'] = df['Fly'][i]
                protocol['classifier'] = {
                        'type'  :  df['Classifier'][i].lower(), 
                        'param' :  df['Classifier Param'][i].lower(),
                        }
                protocol['led_policy'] = {
                        'type'  :  df['LED Policy'][i].lower(), 
                        'param' :  df['LED Policy Param'][i].lower(),
                        }
            else:
                protocol['fly'] = '' 
                protocol['classifier'] = {'type': 'empty', 'param': '{}'}
                protocol['led_policy'] = {'type': 'empty', 'param': '{}'}
            protocol_list.append(protocol)
        self.param['regions']['protocols'] = protocol_list

        # Load trail state and duration parameters
        trial_schedule = []
        for i in range(df.shape[0]):
            if type(df['Trial State'][i]) == str:
                trial_dict = {}
                trial_dict['state'] = df['Trial State'][i]
                trial_dict['duration'] = df['State Duration'][i]
                trial_schedule.append(trial_dict)
            else:
                break
        self.param['trial_schedule'] = trial_schedule


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
            self.tracking_region_list.append(TrackingRegion(region_param,self.devices))

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
        elapsed_time = 0.0 
        self.start_time = rospy.Time.now().to_time()
        self.trial_scheduler.set_state(0,elapsed_time)

        while not rospy.is_shutdown():

            while (self.objects_queue.qsize() > 0):

                # Get current time
                ros_time_now = rospy.Time.now()
                current_time = ros_time_now.to_time()
                elapsed_time = current_time - self.start_time 

                self.trial_scheduler.update(elapsed_time)

                # Process tracked objects
                tracked_objects = self.objects_queue.get()
                self.process_regions(ros_time_now, elapsed_time, tracked_objects)

            # Visualize regions and objecs
            with self.image_lock:
                self.region_visualizer.update(elapsed_time, self.latest_image, self.trial_scheduler)

            if self.trial_scheduler.done:
                # Call ROS shutdown
                break

        self.clean_up()

        if self.param['kill_at_finish']:
            os.system('rosnode kill -a')

    def clean_up(self):
        for tracking_region in self.tracking_region_list: 
            tracking_region.protocol.led_scheduler.led_off()

    def process_regions(self, ros_time_now, elapsed_time, tracked_objects):
        led_enabled = self.trial_scheduler.led_enabled
        msg = PuzzleboxesData()
        msg.header.stamp = ros_time_now
        msg.elapsed_time = elapsed_time
        msg.region_data_list = []
        msg.led_enabled = led_enabled
        for tracking_region in self.tracking_region_list: 
            region_data = tracking_region.update(elapsed_time, tracked_objects, led_enabled)
            msg.region_data_list.append(region_data)
        self.data_pub.publish(msg)
                

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
