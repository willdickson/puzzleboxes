#!/usr/bin/env python
from __future__ import print_function
import cv2
import rospy
from blob_finder import BlobFinder
from puzzleboxes_base import PuzzleBoxesBase
from puzzleboxes_base import TrackedObject 
from puzzleboxes_base import ObjectPosition 

from puzzleboxes.msg import PuzzleboxesData
from puzzleboxes.msg import RegionData


class PuzzleBoxes(PuzzleBoxesBase):

    def __init__(self):
        super(PuzzleBoxes,self).__init__()
        self.blob_finder = BlobFinder( 
                threshold=self.param['tracking']['threshold'],
                minArea=self.param['tracking']['min_area'], 
                maxArea=self.param['tracking']['max_area'],
                )
        self.data_pub = rospy.Publisher('/puzzleboxes_data', PuzzleboxesData, queue_size=10) 

    def process_frame(self,frame_data): 

        ros_time_now = frame_data['ros_time_now']
        current_time = frame_data['current_time']
        elapsed_time = frame_data['elapsed_time']
        image = frame_data['image']
        diff_image = frame_data['diff_image']

        blob_list, blob_image = self.blob_finder.find(diff_image)
        #cv2.imshow('image', image)
        #cv2.imshow('bg', self.bg_image)
        #cv2.imshow('diff', diff_image)
        ##cv2.imshow('blob', blob_image)
        #cv2.waitKey(1)
        ## Devel
        ## -----------------------------------------------------------------------------
        #rospy.logwarn('len(blob_list) = {}'.format(len(blob_list)))
        ## -----------------------------------------------------------------------------

        tracked_objects = []
        for i, blob in enumerate(blob_list):
            obj = TrackedObject()  # Replace this with simple class
            obj.position.x = blob['centroidX']
            obj.position.y = blob['centroidY']
            obj.size = blob['area']
            #rospy.logwarn('  {},  {}'.format(i, blob['area']))
            tracked_objects.append(obj)
        self.process_regions(ros_time_now, elapsed_time, tracked_objects)
        #bgr_image = cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)

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


    def process_regions(self, ros_time_now, elapsed_time, tracked_objects):
        led_enabled = self.trial_scheduler.led_enabled
        msg = PuzzleboxesData()
        msg.header.stamp = ros_time_now
        msg.elapsed_time = elapsed_time
        msg.region_data_list = []
        msg.led_enabled = led_enabled
        msg.queue_overflow = self.queue_overflow
        msg.queue_size = self.image_queue.qsize() 
        for tracking_region in self.tracking_region_list: 
            region_data = tracking_region.update(elapsed_time, tracked_objects, led_enabled)
            msg.region_data_list.append(region_data)
        self.data_pub.publish(msg)

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = PuzzleBoxes()
    node.run()



