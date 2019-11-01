from __future__ import print_function
import rospy
from rect_region import RectRegion
from operator import attrgetter
from protocol import Protocol

from puzzleboxes.msg import RegionData

class TrackingRegion(RectRegion):

    def __init__(self, param, devices):
        super(TrackingRegion,self).__init__(param)
        self.devices = devices
        self.protocol = Protocol(param,devices) 
        self.obj = None

    @property
    def upper_left(self):
        return self.x0, self.y0

    @property
    def lower_right(self):
        return self.x1, self.y1

    @property
    def number(self):
        """ Get region number for display - starts at 1 instead of zero"""
        return self.param['index']+1

    def led_enable(self,value):
        self.protocol.led_enable(value)

    def update(self,t,tracked_obj_list, led_enabled): 
        contained_obj_list = [obj for obj in tracked_obj_list if self.contains(obj)]
        if contained_obj_list:
            self.obj = max(contained_obj_list, key=attrgetter('size'))
        else:
            self.obj = None
        self.protocol.update(t, self.obj, led_enabled)


        msg = RegionData()
        if self.obj is not None:
            msg.object_found = True
            msg.x = self.obj.position.x
            msg.y = self.obj.position.y
        else:
            msg.object_found = False
            msg.x = 0.0
            msg.y = 0.0
        msg.classifier = self.protocol.classifier.state
        msg.led = self.protocol.led_scheduler.state
        return msg






        




