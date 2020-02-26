#!/usr/bin/env python
from __future__ import print_function
import os
import copy
import Queue
import roslib
import rospy
import std_msgs.msg
import cv2
import threading
import pandas as pd
import yaml
import json
import time
import datetime
import numpy as np
import jpeg4py as jpeg

from blob_finder import BlobFinder

from phidgets1031_led_controller import LedController

from tracking_region import TrackingRegion
from region_visualizer import RegionVisualizer
from trial_scheduler import TrialScheduler

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError



class PuzzleBoxesBase(object):

    Default_Param_File = 'puzzleboxes_param.yaml'

    def __init__(self):

        self.devices = {}
        self.devices['led_controller'] = LedController()
        self.image_queue = Queue.Queue()
        self.objects_queue = Queue.Queue()
        self.queue_overflow = False

        rospy.init_node('puzzleboxes')

        # Read parameters
        self.param_path = '/puzzleboxes'
        self.get_param()
        self.check_param()

        # Set Led controller current limit
        current_limit = self.param['default_param']['led_scheduler']['global']['current_limit']
        self.devices['led_controller'].set_current_limit(current_limit)

        self.create_tracking_regions()
        self.trial_scheduler = TrialScheduler(self.param['trial_schedule'])

        self.visualizer_on = True 
        self.single_threaded = True 

        if self.visualizer_on:
            if self.single_threaded:
                self.region_visualizer = RegionVisualizer(self.param)

            else:
                self.region_visualizer_queue = Queue.Queue()
                self.region_visualizer = RegionVisualizer(
                        self.param,
                        self.region_visualizer_queue
                        )
                self.visualizer_thread = threading.Thread(target=self.region_visualizer.run)
                self.visualizer_thread.daemon = True
                self.visualizer_thread.start()

        # Subscribe to camera images
        self.bridge = CvBridge()
        self.image_lock = threading.Lock()
        if self.param['use_compressed_images']:
            self.image_sub = rospy.Subscriber('/camera/image_raw/compressed', CompressedImage, self.compressed_image_callback)
        else:
            self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.ros_image_callback)

        # Load background image
        bg_file_basename, bg_file_ext = os.path.splitext(self.param['regions']['bg_image_file'])
        if bg_file_ext == '.npy':
            self.bg_image = np.load(self.param['regions']['bg_image_file'])
        else:
            bg_image_tmp = cv2.imread(self.param['regions']['bg_image_file'])
            self.bg_image = cv2.cvtColor(bg_image_tmp,cv2.COLOR_BGR2GRAY)

        # Load mask image
        mask_file_basename, mask_file_ext = os.path.splitext(self.param['regions']['mask_image_file'])
        if mask_file_ext == '.npy':
            self.mask_image = np.load(self.param['regions']['mask_image_file'])
        else:
            mask_image_tmp = cv2.imread(self.param['regions']['mask_image_file'])
            self.mask_image = cv2.cvtColor(mask_image_tmp, cv2.COLOR_BGR2GRAY)

        # Erode mask image a bit to remove bright stuff on edges of arenas
        erode_kernel_size = tuple(self.param['regions']['mask_erode']['kernel_size'])
        erode_kernel = np.ones(erode_kernel_size, np.uint8)
        erode_iterations = self.param['regions']['mask_erode']['iterations']
        self.mask_image = cv2.erode(self.mask_image, erode_kernel, iterations=erode_iterations)

        # Create publishers
        self.param_pub = rospy.Publisher('/puzzleboxes_param', std_msgs.msg.String,queue_size=10)
        self.bg_image_pub = rospy.Publisher('/puzzleboxes_bg_image', Image, queue_size=10)
        self.param_pub_count = 0
        self.parm_pub_timer = rospy.Timer(rospy.Duration(secs=self.param['param_pub_period']), self.on_param_pub_timer)


    def on_param_pub_timer(self,event):
        if self.param_pub_count < self.param['param_pub_number']:
            self.param_pub.publish(json.dumps(self.param))
            bg_image_ros = self.bridge.cv2_to_imgmsg(self.bg_image)
            self.bg_image_pub.publish(bg_image_ros)
            self.param_pub_count += 1

    def get_param(self):
        self.param = rospy.get_param(self.param_path, None)
        if self.param is None:
            param_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),self.Default_Param_File)
            rospy.logwarn('puzzleboxes param not on server ... loading file')
            rospy.logwarn('param file = {}'.format(param_file_path))
            with open(param_file_path,'r') as f:
                self.param = yaml.load(f)
        else:
            rospy.logwarn('param file = {}'.format(self.param['trial_param_file'])) 
  
        self.load_region_centers()
        self.load_ledmap()
        self.load_trial_param_from_csv()
        now = datetime.datetime.now()
        self.param['datetime']= now.strftime('%m%d%y_%H%M%S')

    def load_region_centers(self):
        with open(self.param['regions']['centers_file'],'r') as f:
            centers = yaml.load(f)
        self.param['regions']['centers'] = centers

    def load_ledmap(self):
        with open(self.param['regions']['ledmap_file'],'r') as f:
            ledmap = yaml.load(f)
        self.param['regions']['ledmap'] = ledmap

    def load_trial_param_from_csv(self):
        df = pd.read_csv(self.param['trial_param_file'])
        num_centers = len(self.param['regions']['centers'])
        
        #assert df.shape[0] == num_centers, 'number of rows in {} must equal number of centers'.format(self.param['trial_param_file'])

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
                protocol['notes']= df['Notes'][i]

                led_numbers = df['LED Number'][i]
                protocol['led_numbers'] = led_numbers 
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
                    'ledmap'        :  self.param['regions']['ledmap'],
                    'protocol'      :  region_protocol,
                    'center'        :  {'cx': cx, 'cy': cy, }, 
                    'roi'           :  {'x0': x0, 'x1': x1, 'y0': y0, 'y1': y1}, 
                    'default_param' :  self.param['default_param'],
            }
            self.tracking_region_list.append(TrackingRegion(region_param,self.devices))

    def ros_image_callback(self,ros_img): 
        self.image_queue.put(ros_img)

        #cv_img = self.bridge.imgmsg_to_cv2(ros_img,desired_encoding='mono8')
        #self.image_queue.put(cv_img)

    def compressed_image_callback(self,msg): 
        self.image_queue.put(msg)

        #img_cmp = np.fromstring(msg.data, np.uint8)
        #img_bgr = jpeg.JPEG(img_cmp).decode()
        #img_gry = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        #self.image_queue.put(img_gry)

    def run(self):

        elapsed_time = 0.0 
        self.start_time = rospy.Time.now().to_time()
        self.trial_scheduler.set_state(0,elapsed_time)

        frame_proc_count = 0
        queue_get_count = 0

        while not rospy.is_shutdown():

            image_msg = None

            while True:
                try:
                    image_msg = self.image_queue.get_nowait()
                    queue_get_count += 1
                except Queue.Empty:
                    break

            if image_msg is None:
                continue

            dropped_frames = queue_get_count - frame_proc_count - 1

            if self.param['use_compressed_images']:
                image_cmp = np.fromstring(image_msg.data, np.uint8)
                image_bgr = jpeg.JPEG(image_cmp).decode()
                image = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
            else:
                image = self.bridge.imgmsg_to_cv2(image_msg,desired_encoding='mono8')
                image_bgr = cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)

            ros_time_now = rospy.Time.now()
            current_time = ros_time_now.to_time()
            elapsed_time = current_time - self.start_time 

            if self.param['show_processing_dt']:
                if frame_proc_count == 0:
                    last_time = current_time
                else:
                    dt = current_time - last_time 
                    last_time = current_time
                    msg_tuple = (1.0/dt, float(frame_proc_count)/float(queue_get_count), frame_proc_count, queue_get_count, dropped_frames)
                    rospy.logwarn('new_image, {:1.2f}, {:1.3f},  {}, {}, {}'.format(*msg_tuple))

            self.trial_scheduler.update(elapsed_time)

            diff_image = cv2.absdiff(image,self.bg_image)
            masked_diff_image = cv2.bitwise_and(diff_image, self.mask_image)

            frame_data = {
                    'ros_time_now' : ros_time_now,
                    'current_time' : current_time,
                    'elapsed_time' : elapsed_time,
                    'image'        : image,
                    'bgr_image'    : image_bgr,
                    'diff_image'   : masked_diff_image,
                    } 

            #cv2.imshow('diff_image', diff_image)
            #cv2.imshow('mask_image', self.mask_image)
            #cv2.imshow('masked_diff', masked_diff_image)
            #cv2.waitKey(1)

            self.process_frame(frame_data)

            frame_proc_count += 1
            if self.trial_scheduler.done:
                # shutdown
                break

        self.clean_up()

        if self.param['kill_at_finish']:
            os.system('rosnode kill -a')

    def clean_up(self):
        for tracking_region in self.tracking_region_list: 
            tracking_region.protocol.led_scheduler.led_off()

    def process_frame(self,frame_data):
        pass

                

# Utility Classes
# -----------------------------------------------------------------------------------------------------

class TrackedObject(object):

    def __init__(self):
        self.position = ObjectPosition()
        self.size = 0.0

    def __str__(self):
        return 'TrackedObject: position {}, size {}'.format(self.position, self.size)

class ObjectPosition(object):

    def __init__(self):
        self.x = 0
        self.y = 0

    def __str__(self):
        return 'x: {}, y: {}'.format(self.x, self.y)

# -------------------------------------------------------------------------------------------------------
if __name__ == '__main__':

    node = PuzzleBoxesBase()
    node.run()
