from __future__ import print_function
import sys
import cv2
import numpy 
import scipy.ndimage
import skimage.feature 
import skimage.morphology
import skimage.filters
import skimage.measure
import skimage.color

from blob_finder import BlobFinder

class BaseObjectFinder(object):

    def __init__(self,param):
        self.param = param
        self.circle_size = 5 
        self.circle_thickness = 1
        self.blob_finder = BlobFinder( 
                threshold=self.param['tracking']['multiobj_threshold'],
                minArea=self.param['tracking']['min_area'], 
                maxArea=self.param['tracking']['max_area'],
                )


    def update(self, frame_gray):
        blob_list, blob_image = self.blob_finder.find(frame_gray)
        #cv2.imshow('blob_image', blob_image)
        #cv2.waitKey(1)
        return self.find_objects(frame_gray, blob_list) 


class FlyAndOneBallFinder(BaseObjectFinder):
    """
    Currently, for devleopment purposes just designed to work with fly and large ball. 
    """

    def __init__(self,param):
        super(FlyAndOneBallFinder,self).__init__(param)

    def find_objects(self, frame_gray, props_list):
        area_and_props_list = [(item['area'], item) for item in props_list]
        area_and_props_list.sort(reverse=True)
        props_list = [props for (area,props) in area_and_props_list]

        if len(props_list) > 2:
            props_list = props_list[:2]

        obj_props_dict = {}

        if len(props_list) == 2:
            obj_props_dict['fly'] = props_list[1] 
            obj_props_dict['ball'] = props_list[0] 

        if len(props_list) == 1:
            obj_props_dict['fly'] = props_list[0] 
            obj_props_dict['ball'] = props_list[0] 

        return obj_props_dict


class FlyAndTwoBallFinder(BaseObjectFinder):

    """ WARNING:  probably broken because of change in object finder """
    
    def __init__(self):
        super(FlyAndTwoBallFinder,self).__init__()
        self.obj_stats_dict = { 
                'small_ball': ObjectStats(),
                'large_ball': ObjectStats(),
                'fly'       : ObjectStats(),
                }
        self.color_dict = {
                'fly'       : (  0,   0, 255),
                'small_ball': (255,   0,   0),
                'large_ball': (  0, 255,   0),
                }

    def find_objects(self, frame_gray, props_list):

        frame_viz = cv2.cvtColor(frame_gray, cv2.COLOR_GRAY2BGR)

        if len(props_list) == 3:

            # Find fly
            ax_ratio_list = [(p.major_axis_length/p.minor_axis_length,p) for p in props_list]
            ax_ratio_list.sort()
            ax_ratio_list.reverse()
            fly_props = ax_ratio_list[0][1]
            self.obj_stats_dict['fly'].update(fly_props)

            # Find big and small balls
            ball_props_list = [item[1] for item in ax_ratio_list[1:]]
            area_list = [(p.area, p) for p in ball_props_list]
            area_list.sort()
            ball_props_list = [item[1] for item in area_list]

            # Plot fly
            fly_centroid = (int(numpy.round(fly_props.centroid[1])), int(numpy.round(fly_props.centroid[0])))
            cv2.circle(frame_viz,fly_centroid,self.circle_size,self.color_dict['fly'],self.circle_thickness) 

            # Plot small and big balls
            ball_color_list = [self.color_dict['small_ball'], self.color_dict['large_ball']]
            ball_stats_list = [self.obj_stats_dict['small_ball'], self.obj_stats_dict['large_ball']]
            for props, color, ball_stats in zip(ball_props_list, ball_color_list, ball_stats_list):
                ball_stats.update(props)
                ball_centroid = (int(numpy.round(props.centroid[1])), int(numpy.round(props.centroid[0])))
                cv2.circle(frame_viz,ball_centroid,self.circle_size,color,self.circle_thickness)

        if len(props_list) == 2:

            obj_dist_list = []#
            for i, p in enumerate(props_list):
                for name, stats in self.obj_stats_dict.items():
                    dist = object_dist(stats, ObjectStats(p))
                    obj_dist_list.append((dist, i, name))
            obj_dist_list.sort()

            single_item_index = obj_dist_list[0][1]
            single_item_name = obj_dist_list[0][2]

            pair_item_name_list = [k for k in self.obj_stats_dict if k!=single_item_name]
            pair_item_index = (single_item_index + 1) % 2 

            single_item_props = props_list[single_item_index]
            single_item_centroid =  (int(numpy.round(single_item_props.centroid[1])), int(numpy.round(single_item_props.centroid[0])))
            cv2.circle(frame_viz,single_item_centroid,self.circle_size,self.color_dict[single_item_name],self.circle_thickness) 

            pair_item_props = props_list[pair_item_index]
            pair_item_centroid =  (int(numpy.round(pair_item_props.centroid[1])), int(numpy.round(pair_item_props.centroid[0])))
            for i, item_name in enumerate(pair_item_name_list):
                cv2.circle(frame_viz,pair_item_centroid,self.circle_size+i,self.color_dict[item_name],self.circle_thickness) 

        if len(props_list) == 1:
            item_props = props_list[0]
            item_centroid =  (int(numpy.round(item_props.centroid[1])), int(numpy.round(item_props.centroid[0])))
            for i, item_name in enumerate(self.obj_stats_dict.keys()):
                cv2.circle(frame_viz,item_centroid,self.circle_size+i,self.color_dict[item_name]) 

        cv2.imshow('frame_viz', frame_viz)



class ObjectProp(object):

    def __init__(self,value=None): 
        self.count = 0
        self.mean = 0.0
        self.variance = 0.0
        self.max = None
        self.min = None
        if value is not None:
                self.update(value)

    def print(self,pad=''):
        print('{}mean: {:1.3f}, std:  {:1.3f}, max: {:1.3f}, min: {:1.3f}'.format(pad, self.mean, self.std, self.max, self.min))

    @property
    def std(self):
        return numpy.sqrt(self.variance)

    def update(self,value):
        if self.count == 0:
            self.mean = value
        else:
            k0 = 1.0/float(self.count+1)
            k1 = float(self.count)/float(self.count+1)
            self.mean = k0*value + k1*self.mean
            self.variance = k0*(value - self.mean)**2 + k1*self.variance
        if self.max is None:
            self.max = value
        else:
            self.max = max(value, self.max)
            self.min = min(value, self.min)
        if self.min is None:
            self.min = value
        self.count += 1

            
class ObjectStats(object):

    def __init__(self,props=None):
        self.update_count = 0
        self.area = ObjectProp()
        self.major_axis_length = ObjectProp()
        self.minor_axis_length = ObjectProp()
        self.aspect_ratio = ObjectProp()
        self.attr_name_list = ['area', 'major_axis_length', 'minor_axis_length', 'aspect_ratio']
        if props is not None:
            self.update(props)

    def print(self, pad=''):
        for i, attr_name in enumerate(self.attr_name_list): 
            print('{}{}'.format(pad,attr_name))
            attr = getattr(self,attr_name)
            attr.print(pad=2*pad)
            if i < len(self.attr_name_list) -1:
                print()

    def update(self,props):
        self.area.update(props.area)
        self.major_axis_length.update(props.major_axis_length)
        self.minor_axis_length.update(props.minor_axis_length)
        if props.minor_axis_length > 0:
            self.aspect_ratio.update(props.major_axis_length/props.minor_axis_length)
        self.update_count+=1


def object_dist(x,y):
    diff_area = numpy.absolute(x.area.mean - y.area.mean)
    major_axis_diff_sqr = (x.major_axis_length.mean - y.major_axis_length.mean)**2
    minor_axis_diff_sqr = (x.minor_axis_length.mean - y.minor_axis_length.mean)**2
    return numpy.sqrt(diff_area + major_axis_diff_sqr + minor_axis_diff_sqr)


