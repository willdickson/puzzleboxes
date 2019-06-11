import cv2
import numpy as np
import yaml
#from PIL import ImageFont, ImageDraw, Image

class RegionVisualizer(object):

    def __init__(self,tracking_region_list):
        self.tracking_region_list = tracking_region_list
        self.is_first_image = True
        self.window_name = 'tracking_regions'
        self.window_num_pad = 1,11
                
        # color palette BGR
        self.red =  (18,  18, 209)
        self.green = ( 0, 153,  30)
        self.yellow = (0,200,255)
        self.white = (200, 200, 200)
        self.gray = (120, 120, 120)
        
        # Mapping from (classifier.state, led.state) to object color
        self.states_to_object_color = {
                (False, False) : self.white,
                (True,  False) : self.green,
                (False, True)  : self.red,
                (True,  True)  : self.red,
                }
                
        self.object_radius = 7
        self.object_linewidth_thin = 1
        self.object_linewidth_thick = 2
        self.classifier_color = self.yellow
        self.classifier_thickness = 2

        self.find_centers = False
        self.centers_count = 0

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.fontsize = 0.3
        #self.PILfont = ImageFont.truetype('/home/flyranch-corfas/catkin_ws/src/puzzleboxes/fonts/visitor2.ttf',12)
        
    def update(self, image):
        if image is None:
            return

        if self.is_first_image: 
            # Setup opencv plotting windows
            cv2.namedWindow(self.window_name,cv2.WINDOW_NORMAL)
            cv2.moveWindow(self.window_name, 200, 250)
            cv2.resizeWindow(self.window_name, 800,600)
            self.is_first_image = False 
            if self.find_centers:
                cv2.namedWindow('centers_image',cv2.WINDOW_NORMAL)
                cv2.moveWindow('centers_image', 120, 120)
                cv2.resizeWindow('centers_image', 800,600)
        
        for tracking_region in self.tracking_region_list: 
            self.annotate_region(image,tracking_region)
            self.draw_classifier(image,tracking_region)
            self.draw_object(image, tracking_region)
        
        # self.print_elapsed_time(image, t)
        self.display_trial_state(image)
        
        cv2.imshow(self.window_name,image)
        cv2.waitKey(1)

    def print_elapsed_time(self, image, t):
        elapsed_t = t
        cv2.putText(image,'{}'.format(elapsed_t), (10,10), self.font, self.fontsize, self.yellow)
        
    def display_trial_state(self, image):
        trial_state_name = 'ENABLED' #UPDATE WITH DYNAMIC VERSION
        if 'ENABLED' in trial_state_name:
            display_color = self.red
        else:
            display_color = self.gray
        cv2.putText(image,'{}'.format(trial_state_name), (570,12), self.font, self.fontsize*1.3, display_color)
    
    
    def annotate_region(self, image, tracking_region):
        x0, y0 = tracking_region.upper_left
        x1, y1 = tracking_region.lower_right
        
        # annotate led type (and set annotation color)
        led_scheduler_type = tracking_region.protocol.led_scheduler.type
        if led_scheduler_type == 'instant' or led_scheduler_type == 'pulse':
            led_scheduler_name = 'PULSE' #UPDATE WITH DYNAMIC VERSION
            annotation_color = self.yellow
            text_size = cv2.getTextSize(led_scheduler_name, self.font, self.fontsize,1)[0]
            tx = x1 - text_size[0]
            ty = y1 - 2
            cv2.putText(image,'{}'.format(led_scheduler_name), (tx,ty), self.font, self.fontsize, annotation_color)
        else:
            annotation_color = self.gray
        #if led_scheduler_type == 'yoked':
        #    # Annotate yoked region
        #    tx = x1 - self.window_num_pad[0]
        #    ty = y1 - self.window_num_pad[1]
        #    cv2.putText(image,'{}'.format(yoked_region), (tx,ty), self.font, self.fontsize, self.red)
        
        # draw bounding box on all regions with classifier and annotate classifier type and genotype
        classifier_type = tracking_region.protocol.classifier.type
        if classifier_type != 'empty':
            cv2.rectangle(image, (x0,y0), (x1,y1), annotation_color, 1)
            # Annotate region number
            tx = x0 + 1
            ty = y0 + 8
            cv2.putText(image,'{}'.format(tracking_region.number), (tx,ty), self.font, self.fontsize, annotation_color)
            # Annotate classifier name
            classifier_name = 'CTR' #UPDATE WITH DYNAMIC VERSION
            tx = x0 + 1
            ty = y1 - 2
            cv2.putText(image,'{}'.format(classifier_name), (tx,ty), self.font, self.fontsize, annotation_color)
            # annotate fly genotype
            genotype_name = 'Gr43a' #UPDATE WITH DYNAMIC VERSION
            text_size = cv2.getTextSize(genotype_name, self.font, self.fontsize,1)[0]
            tx = x1 - text_size[0]
            ty = y0 + 8
            cv2.putText(image,'{}'.format(genotype_name), (tx,ty), self.font, self.fontsize, annotation_color)
            
            #cv2_im_rgb = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
            #pil_im = Image.fromarray(cv2_im_rgb)
            #draw = ImageDraw.Draw(pil_im)
            #draw.text((83,199), genotype_name, font=self.PILfont)
            #image = cv2.cvtColor(np.array(pil_im),cv2.COLOR_RGB2BGR)
            #cv2.imshow('test',image)

    def draw_object(self, image, tracking_region): 
        if tracking_region.obj is not None:
            x = int(tracking_region.obj.position.x)
            y = int(tracking_region.obj.position.y)
            classifier_state = tracking_region.protocol.classifier.state
            led_state = tracking_region.protocol.led_scheduler.state
            object_color = self.states_to_object_color[(classifier_state,led_state)]
            if led_state or classifier_state:
                object_linewidth = self.object_linewidth_thick
            else:
                object_linewidth = self.object_linewidth_thin
            cv2.circle(image, (x,y), self.object_radius, object_color, object_linewidth)


    def draw_classifier(self, image, tracking_region):
        classifier_type = tracking_region.protocol.classifier.type
        classifier_param = tracking_region.protocol.classifier.classifier_param
        if classifier_type == 'center':
            cx = tracking_region.param['center']['cx']
            cy = tracking_region.param['center']['cy']
            if 'radius' in classifier_param:
                radius = classifier_param['radius']
                cv2.circle(image, (cx,cy), radius, self.classifier_color,self.classifier_thickness)
            elif 'height' in classifier_param:
                height = classifier_param['height']
                width = classifier_param['width']
                bottom_left = (cx-width/2,cy+height/2)
                top_right = (cx+width/2,cy-height/2)
                cv2.rectangle(image, bottom_left, top_right, self.classifier_color,self.classifier_thickness)
        elif classifier_type == 'empty':
            pass







# SAVE THIS !!!!! 
# ---------------------------------------------------------------------------------------------------------------
#    def update(self,image):
#        """
#        Extraced from update above ...
#    
#        """
#            if self.find_centers:
#                # Temporary: move to another node which is run during setup
#                centers_image = self.find_centers_from_image(image)
#                cv2.imshow('centers_image', centers_image)
#
#    def find_centers_from_image(self, image):
#        """
#        Temporary: move to some setup node etc.
#        """
#        threshold = 30
#        min_area = 100
#        image_gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
#        rval, threshold_image = cv2.threshold(image_gray, threshold, np.iinfo(image.dtype).max, cv2.THRESH_BINARY)
#        dummy, contour_list, dummy = cv2.findContours(threshold_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
#        centroid_list = []
#        filtered_contour_list = []
#        for contour in contour_list:
#            moments = cv2.moments(contour) 
#            if moments['m00'] > 0: 
#                centroidX = moments['m10']/moments['m00'] 
#                centroidY = moments['m01']/moments['m00']
#            else:
#                continue
#            area = cv2.contourArea(contour)
#            if area > min_area:
#                filtered_contour_list.append(contour)
#                centroid_list.append([int(centroidX),int(centroidY)])
#                
#        contours_image = cv2.cvtColor(image_gray,cv2.COLOR_GRAY2BGR)
#        cv2.drawContours(contours_image,filtered_contour_list,-1,(255,255,255),2)
#
#        for cx,cy in centroid_list:
#            cv2.circle(contours_image,(cx,cy),3,(255,255,255))
#
#        self.centers_count += 1
#        if self.centers_count == 50:
#            print('writing centers')
#            centroid_list.sort()   # This doesn't really work
#            with open('centers.yaml','w') as f:
#                yaml.dump(centroid_list, f)
#
#        return contours_image 
