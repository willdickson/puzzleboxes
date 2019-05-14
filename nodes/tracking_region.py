from __future__ import print_function
import rospy
from rect_region import RectRegion
from operator import attrgetter

#from path_integration_3x3.msg import TrackingRegionData

class TrackingRegion(RectRegion):

    def __init__(self, region_index, params):
        super(TrackingRegion,self).__init__(params)
        self.index = region_index
        self.object = None

        #self.led_region_list = []
        #for led_index, led_base_param in enumerate(self.params['leds']):
        #    led_param = dict(params['stimulation'])
        #    led_param.update(led_base_param)
        #    self.led_region_list.append(LedRegion(region_index,led_index,led_param))

    @property
    def x0(self):
        return self.param['x0']

    @property
    def x1(self):
        return self.param['x1']

    @property
    def y0(self):
        return self.param['y0']

    @property
    def y1(self):
        return self.param['y1']

    @property
    def lower_left(self):
        return self.x0, self.y0

    @property
    def upper_right(self):
        return self.x1, self.y1

    @property
    def number(self):
        """ Get region number for display - starts at 1 instead of zero"""
        return self.index+1

    def update(self,t,tracked_objects): 
        contained_objects = [obj for obj in tracked_objects if self.contains(obj)]
        #region_msg = TrackingRegionData()
        #region_msg.index = self.index 
        if contained_objects:
            self.object = max(contained_objects, key=attrgetter('size'))
            #print('object found: region {}, position {}'.format(self.number,self.object.position))
            #region_msg.object_found = True 
            #region_msg.object = max_object 
            #for led_region in self.led_region_list:
            #    led_data = led_region.update(t, max_object)
            #    region_msg.led_region_data.append(led_data)
        else:
            self.object = None
            #region_msg.object_found = False
        #return region_msg





        




