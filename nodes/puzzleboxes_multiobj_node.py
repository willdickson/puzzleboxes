#!/usr/bin/env python
from __future__ import print_function
import cv2
import rospy
from puzzleboxes_base import PuzzleBoxesBase

class PuzzleBoxesMultiObj(PuzzleBoxesBase):

    def __init__(self):
        super(PuzzleBoxesMultiObj,self).__init__()

    def process_frame(self,frame_data): 

        ros_time_now = frame_data['ros_time_now']
        current_time = frame_data['current_time']
        elapsed_time = frame_data['elapsed_time']
        image = frame_data['image']
        diff_image = frame_data['diff_image']

        region_image_list = self.get_region_images(diff_image)

        rospy.logwarn('w: {}'.format(self.param['regions']['width']))
        rospy.logwarn('h: {}'.format(self.param['regions']['height']))
        rospy.logwarn('elasped time: {:1.2f}'.format(elapsed_time))

        cv2.imshow('diff_image', diff_image)
        cv2.imshow('region_image', region_image_list[19])
        cv2.waitKey(1)


    def get_region_images(self,diff_image):
        image_list = []
        for region in self.tracking_region_list:
            p = region.lower_right
            q = region.upper_left
            rospy.logwarn('p: {}, q: {}'.format(p,q))
            diff_image_crop = diff_image[q[1]:p[1], q[0]:p[0]]
            image_list.append(diff_image_crop)
        return image_list


            



# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = PuzzleBoxesMultiObj()
    node.run()



