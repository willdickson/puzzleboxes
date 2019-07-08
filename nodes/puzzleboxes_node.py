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

from blob_finder import BlobFinder

from phidgets1031_led_controller import LedController

from tracking_region import TrackingRegion
from region_visualizer import RegionVisualizer
from trial_scheduler import TrialScheduler

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from puzzleboxes.msg import PuzzleboxesData
from puzzleboxes.msg import RegionData


class PuzzleBoxes(object):

    Default_Param_File = 'puzzleboxes_param.yaml'

    def __init__(self,nodenum=1):

        self.devices = {}
        self.devices['led_controller'] = LedController()
        self.image_queue = Queue.Queue()
        self.objects_queue = Queue.Queue()

        rospy.init_node('puzzleboxes')

        # Read parameters
        self.param_path = '/puzzleboxes'
        self.get_param()
        self.check_param()

        self.create_tracking_regions()
        self.region_visualizer = RegionVisualizer(self.tracking_region_list)
        self.trial_scheduler = TrialScheduler(self.param['trial_schedule'])

        # Subscribe to camera images
        self.bridge = CvBridge()
        self.image_lock = threading.Lock()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

        # Load background image
        self.bg_image = np.load(self.param['regions']['bg_image_file'])

        # Create publishers
        self.data_pub = rospy.Publisher('/puzzleboxes_data', PuzzleboxesData, queue_size=10) 
        self.param_pub = rospy.Publisher('/puzzleboxes_param', std_msgs.msg.String,queue_size=10)
        self.bg_image_pub = rospy.Publisher('/puzzleboxes_bg_image', Image, queue_size=10)
        self.parm_pub_timer = rospy.Timer(rospy.Duration(secs=self.param['param_pub_period']), self.on_param_pub_timer)


    def on_param_pub_timer(self,event):
        self.param_pub.publish(json.dumps(self.param))
        bg_image_ros = self.bridge.cv2_to_imgmsg(self.bg_image)
        self.bg_image_pub.publish(bg_image_ros)

    def get_param(self):
        self.param = rospy.get_param(self.param_path, None)
        if self.param is None:
            param_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),self.Default_Param_File)
            rospy.logwarn('puzzleboxes param not on server ... loading file')
            rospy.logwarn('param file = {}'.format(param_file_path))
            with open(param_file_path,'r') as f:
                self.param = yaml.load(f)

        self.load_region_centers()
        self.load_trial_param_from_csv()
        now = datetime.datetime.now()
        self.param['datetime']= now.strftime('%m%d%y_%H%M%S')

    def load_region_centers(self):
        with open(self.param['regions']['centers_file'],'r') as f:
            centers = yaml.load(f)
        self.param['regions']['centers'] = centers

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

        # Load trial state and duration parameters
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
        self.image_queue.put(cv_img)

    def run(self):

        elapsed_time = 0.0 
        self.start_time = rospy.Time.now().to_time()
        self.trial_scheduler.set_state(0,elapsed_time)

        blob_finder = BlobFinder(
                threshold=self.param['tracking']['threshold'],
                minArea=self.param['tracking']['min_area'], 
                maxArea=self.param['tracking']['max_area'],
                )

        while not rospy.is_shutdown():

            while (self.image_queue.qsize() > 0):

                ros_time_now = rospy.Time.now()
                current_time = ros_time_now.to_time()
                elapsed_time = current_time - self.start_time 

                image = self.image_queue.get()
                diff_image = cv2.absdiff(image,self.bg_image)
                blob_list, blob_image = blob_finder.find(diff_image)
                tracked_objects = []
                for blob in blob_list:
                    obj = TrackedObject()  # Replace this with simple class
                    obj.position.x = blob['centroidX']
                    obj.position.y = blob['centroidY']
                    obj.size = blob['area']
                    tracked_objects.append(obj)
                self.trial_scheduler.update(elapsed_time)
                self.process_regions(ros_time_now, elapsed_time, tracked_objects)
                bgr_image = cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
                self.region_visualizer.update(elapsed_time, bgr_image, self.trial_scheduler)

            if self.trial_scheduler.done:
                # Call ROS shutdown
                break

        self.clean_up()

        #if self.param['kill_at_finish']:
        #    os.system('rosnode kill -a')

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
                

# Utility Classes
# -----------------------------------------------------------------------------------------------------

class TrackedObject(object):

    def __init__(self):
        self.position = ObjectPosition()

    def __str__(self):
        return 'TrackedObject: {}'.format(self.position)

class ObjectPosition(object):

    def __init__(self):
        self.x = 0
        self.y = 0

    def __str__(self):
        return 'x: {}, y: {}'.format(self.x, self.y)

# -------------------------------------------------------------------------------------------------------
if __name__ == '__main__':

    node = PuzzleBoxes()
    node.run()
