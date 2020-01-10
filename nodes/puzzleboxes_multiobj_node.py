#!/usr/bin/env python
from __future__ import print_function
import cv2
import rospy
import time
from puzzleboxes_base import PuzzleBoxesBase
from tracking_region import FlyAndOneBallTrackingRegion
from object_finder import FlyAndTwoBallFinder 
from object_finder import FlyAndOneBallFinder 

from puzzleboxes.msg import PuzzleboxesFlyAndOneBallData
from puzzleboxes.msg import FlyAndOneBallRegionData

class PuzzleBoxesMultiObj(PuzzleBoxesBase):

    def __init__(self):
        super(PuzzleBoxesMultiObj,self).__init__()
        self.upgrade_tracking_regions()
        self.object_finder = FlyAndOneBallFinder(self.param)
        self.data_pub = rospy.Publisher('/puzzleboxes_data', PuzzleboxesFlyAndOneBallData, queue_size=10) 

    def upgrade_tracking_regions(self):
        new_tracking_region_list = []
        for region in self.tracking_region_list:
            new_tracking_region = FlyAndOneBallTrackingRegion(region)
            new_tracking_region_list.append(new_tracking_region)
        self.tracking_region_list = new_tracking_region_list

    def process_frame(self,frame_data): 

        ros_time_now = frame_data['ros_time_now']
        current_time = frame_data['current_time']
        elapsed_time = frame_data['elapsed_time']
        image = frame_data['image']
        diff_image = frame_data['diff_image']

        region_image_list = self.get_region_images(diff_image)
        #t0 = time.time()
        self.process_regions(ros_time_now, elapsed_time, region_image_list)
        #t1 = time.time()
        #dt = t1 - t0
        #rospy.logwarn('dt = {:1.4f}'.format(dt))

        if self.visualizer_on:
            visualizer_data = {
                    'elapsed_time'         : elapsed_time,
                    'bgr_image'            : frame_data['bgr_image'],
                    'trial_scheduler'      : self.trial_scheduler,
                    'tracking_region_list' : self.tracking_region_list,
                    }

            if self.single_threaded:
                self.region_visualizer.update(visualizer_data)
            else:
                self.region_visualizer_queue.put(visualizer_data)


    def process_regions(self, ros_time_now, elapsed_time, region_image_list):
        led_enabled = self.trial_scheduler.led_enabled
        msg = PuzzleboxesFlyAndOneBallData()
        msg.header.stamp = ros_time_now
        msg.elapsed_time = elapsed_time
        msg.region_data_list = []
        msg.led_enabled = led_enabled
        msg.queue_overflow = self.queue_overflow
        msg.queue_size = self.image_queue.qsize() 
        for region, region_image in zip(self.tracking_region_list,region_image_list): 
            if True:
            #if region.param['index'] == 20:
                obj_dict = self.object_finder.update(region_image)
                region_data = region.update(elapsed_time, obj_dict, led_enabled)
                msg.region_data_list.append(region_data)
        self.data_pub.publish(msg)


    def get_region_images(self,diff_image):
        image_list = []
        for region in self.tracking_region_list:
            p = region.lower_right
            q = region.upper_left
            diff_image_crop = diff_image[q[1]:p[1], q[0]:p[0]]
            image_list.append(diff_image_crop)
        return image_list



# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = PuzzleBoxesMultiObj()
    node.run()



