import sys
import cv2
import numpy 
import scipy.ndimage
import skimage.feature 
import skimage.morphology
import skimage.filters
import skimage.measure
import skimage.color

method = 'test'
wait_key_dt = 1

color_dict = {
        'fly'       : (  0,   0, 255),
        'small_ball': (255,   0,   0),
        'large_ball': (  0, 255,   0),
        }

fly_color = (0,0,255)
small_ball_color = (255, 0, 0)
large_ball_color = (0, 255, 0)
circle_size = 5 
circle_thickness = 1

videofile = sys.argv[1]
cap = cv2.VideoCapture(videofile)

outfile = 'example.avi'
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = None


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



obj_stats_dict = {
        'small_ball': ObjectStats(),
        'large_ball': ObjectStats(),
        'fly':        ObjectStats(),
        }


count = 0
while True:
    ret, frame = cap.read()
    if not ret:
        break
    frame_gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # Watershed blob detection
    # see https://www.pyimagesearch.com/2015/11/02/watershed-opencv/
    # ----------------------------------------------------------------------------------------
    if method == 'test':
        if count == 0:
            cv2.namedWindow('frame_orig', cv2.WINDOW_NORMAL)
            cv2.namedWindow('frame_thresh', cv2.WINDOW_NORMAL)
            #cv2.namedWindow('frame_labels', cv2.WINDOW_NORMAL)

        thresh_man = 35 
        #thresh_ostu = skimage.filters.threshold_otsu(frame_gray)
        #thresh_yen = skimage.filters.threshold_yen(frame_gray)
        #thresh_li = skimage.filters.threshold_li(frame_gray)
        #thresh_niblack = skimage.filters.threshold_niblack(frame_gray,window_size=11,k=-0.5)
        #thresh_tri = skimage.filters.threshold_triangle(frame_gray)
        #thresh_iso = skimage.filters.threshold_isodata(frame_gray)

        #frame_mask = frame_gray > thresh_ostu 
        #frame_mask = frame_gray > thresh_yen 
        #frame_mask = frame_gray > thresh_li 
        #frame_mask = frame_gray > thresh_niblack 
        #frame_mask = frame_gray > thresh_tri 
        #frame_mask = frame_gray > thresh_iso
        frame_mask = frame_gray > thresh_man
        frame_thresh = numpy.zeros(frame_gray.shape, dtype=numpy.uint8)
        frame_thresh[frame_mask] = 255

        morph_kernel = numpy.ones((3,3))
        frame_thresh = skimage.morphology.dilation(frame_thresh,morph_kernel)
        frame_thresh = skimage.morphology.erosion(frame_thresh,morph_kernel)


        #frame_thresh = skimage.morphology.area_opening(frame_thresh, area_threshold=5)
        #frame_thresh = skimage.morphology.area_closing(frame_thresh, area_threshold=10)


        frame_labels = skimage.measure.label(frame_thresh)
        frame_color_labels = skimage.color.label2rgb(frame_labels)

        # Get region properties and remove degenerate cases
        props_list = skimage.measure.regionprops(frame_labels)
        props_list = [p for p in props_list if (p.major_axis_length > 0 and p.minor_axis_length > 0)] 

        if len(props_list) == 3:

            # Find fly
            ax_ratio_list = [(p.major_axis_length/p.minor_axis_length,p) for p in props_list]
            ax_ratio_list.sort()
            ax_ratio_list.reverse()
            fly_props = ax_ratio_list[0][1]
            obj_stats_dict['fly'].update(fly_props)

            # Find big and small balls
            ball_props_list = [item[1] for item in ax_ratio_list[1:]]
            area_list = [(p.area, p) for p in ball_props_list]
            area_list.sort()
            ball_props_list = [item[1] for item in area_list]

            # Plot fly
            fly_centroid = (int(numpy.round(fly_props.centroid[1])), int(numpy.round(fly_props.centroid[0])))
            cv2.circle(frame,fly_centroid,circle_size,color_dict['fly'],circle_thickness) 

            # Plot small and big balls
            ball_color_list = [color_dict['small_ball'], color_dict['large_ball']]
            ball_stats_list = [obj_stats_dict['small_ball'], obj_stats_dict['large_ball']]
            for props, color, ball_stats in zip(ball_props_list, ball_color_list, ball_stats_list):
                ball_stats.update(props)
                ball_centroid = (int(numpy.round(props.centroid[1])), int(numpy.round(props.centroid[0])))
                cv2.circle(frame,ball_centroid,circle_size,color,circle_thickness)

        if len(props_list) == 2:

            obj_dist_list = []#
            for i, p in enumerate(props_list):
                for name, stats in obj_stats_dict.items():
                    dist = object_dist(stats, ObjectStats(p))
                    obj_dist_list.append((dist, i, name))
            obj_dist_list.sort()

            single_item_index = obj_dist_list[0][1]
            single_item_name = obj_dist_list[0][2]

            pair_item_name_list = [k for k in obj_stats_dict if k!=single_item_name]
            pair_item_index = (single_item_index + 1) % 2 

            single_item_props = props_list[single_item_index]
            single_item_centroid =  (int(numpy.round(single_item_props.centroid[1])), int(numpy.round(single_item_props.centroid[0])))
            cv2.circle(frame,single_item_centroid,circle_size,color_dict[single_item_name],circle_thickness) 

            pair_item_props = props_list[pair_item_index]
            pair_item_centroid =  (int(numpy.round(pair_item_props.centroid[1])), int(numpy.round(pair_item_props.centroid[0])))
            for i, item_name in enumerate(pair_item_name_list):
                cv2.circle(frame,pair_item_centroid,circle_size+i,color_dict[item_name],circle_thickness) 

        if len(props_list) == 1:
            item_props = props_list[0]
            item_centroid =  (int(numpy.round(item_props.centroid[1])), int(numpy.round(item_props.centroid[0])))
            for i, item_name in enumerate(obj_stats_dict.keys()):
                cv2.circle(frame,pair_item_centroid,circle_size+i,color_dict[item_name]) 
            
        print('{}'.format(count))
        print()
        for k,v in obj_stats_dict.items():
            print(k)
            print('-'*len(k))
            print()
            v.print(pad=' ')
            print()

        n,m,k = frame.shape
        if out is None:
            out = cv2.VideoWriter(outfile, fourcc, 30.0, (m,n))
        out.write(frame)

        #print(obj_stats_dict['small_ball'].area.mean/obj_stats_dict['fly'].area.mean)
        print()
        cv2.imshow('frame_orig', frame)
        cv2.imshow('frame_thresh', frame_thresh)
        #cv2.imshow('frame_labels', frame_color_labels)
        if cv2.waitKey(wait_key_dt) & 0xff == ord('q'):
            break
        count += 1



cap.release()
cv2.destroyAllWindows()
