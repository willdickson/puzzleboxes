from __future__ import print_function
import os
import cv2
import numpy as np
import yaml
import rospy

class RegionVisualizer(object):

    def __init__(self,tracking_region_list, param):
        self.tracking_region_list = tracking_region_list
        self.param = param
        self.is_first_image = True
        self.window_name = 'tracking_regions'
        self.window_num_pad = 1,11
                
        # color palette BGR
        self.red =  (3,  3, 191)
        self.green = ( 0, 124,  25)
        self.yellow = (0,170,255)
        self.white = (200, 200, 200)
        self.gray = (80, 80, 80)
        
        # Mapping from (classifier.state, led.state) to object color
        self.states_to_object_color = {
                (False, False) : self.gray,
                (True,  False) : self.green,
                (False, True)  : self.red,
                (True,  True)  : self.red,
                }
                
        self.object_radius = 30 
        self.object_linewidth_thin = 3
        self.object_linewidth_thick = 5
        self.classifier_color = self.yellow
        self.classifier_off_color = self.white
        self.classifier_thickness = 2
        self.bounding_box_linewidth = 2

        self.find_centers = False
        self.centers_count = 0

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.fontsize = 1.0
        self.fontsize_large = 1.5
        self.fontthickness = 2
        self.fontthickness_thick = 3
        self.text_offset = 5
        
    def update(self, elapsed_time, image, trial_scheduler):
        if image is None:
            return

        if self.is_first_image: 
            # Setup opencv plotting windows
            cv2.namedWindow(self.window_name,cv2.WINDOW_AUTOSIZE)
            cv2.moveWindow(self.window_name, 100, 100)
            #cv2.resizeWindow(self.window_name, 800,600)
            self.is_first_image = False 
            if self.find_centers:
                cv2.namedWindow('centers_image',cv2.WINDOW_NORMAL)
                #cv2.moveWindow('centers_image', 120, 120)
                #cv2.resizeWindow('centers_image', 800,600)
        
        self.display_text = {}
                
        for tracking_region in self.tracking_region_list:
            color = self.determine_region_color(tracking_region)
            self.annotate_region(image,tracking_region, self.display_text, color)
            self.draw_bounding_box(image,tracking_region, color)
            self.draw_classifier(image,tracking_region)
            self.draw_object(image, tracking_region)

        self.draw_display_text(image, elapsed_time, trial_scheduler, self.display_text)

        if self.param['use_compressed_images']:
            n,m,c = image.shape
            ns, ms = int(n*self.param['display_scale']), int(m*self.param['display_scale'])
            image = cv2.resize(image,(ms,ns),0,0,cv2.INTER_AREA)
        
        cv2.imshow(self.window_name,image)
        cv2.waitKey(1)
    
    def determine_region_color(self, tracking_region):
        color = self.gray # default
        led_scheduler_type = tracking_region.protocol.led_scheduler.type
        genotype_name = tracking_region.protocol.param['protocol']['fly']
        led_active = False
        control_fly = False
        if led_scheduler_type == 'instant' or led_scheduler_type == 'pulse':
            led_active = True
        if 'HCS' in genotype_name or '+' in genotype_name:
            control_fly = True
        if led_active and not control_fly:
            color = self.yellow
        return color
        
    def annotate_region(self, image, tracking_region, display_text, color):
        x0, y0 = tracking_region.upper_left
        x1, y1 = tracking_region.lower_right
        arena = tracking_region.number
        led_scheduler_type = tracking_region.protocol.led_scheduler.type
        genotype_name = tracking_region.protocol.param['protocol']['fly']
        
        display_text[arena]={
            'led'           :   {},
            'classifier'    :   {},
            'fly'           :   {},
            'region'        :   {},
            }
            
        # annotate led type
        if led_scheduler_type == 'instant' or led_scheduler_type == 'pulse':
            led_scheduler_name = tracking_region.protocol.led_scheduler.led_scheduler_param['display_name']
            display_text[arena]['led']['text'] = led_scheduler_name
            text_size = cv2.getTextSize(led_scheduler_name, self.font, self.fontsize, self.fontthickness)
            display_text[arena]['led']['font'] = self.font
            display_text[arena]['led']['posX'] = x1 - text_size[0][0] - self.text_offset
            display_text[arena]['led']['posY'] = y1 - self.text_offset
            display_text[arena]['led']['color'] = color
        #if led_scheduler_type == 'yoked':
        #    cv2.putText(image,'{}'.format(yoked_region), (tx,ty), self.font, self.fontsize, self.red)
        
        # Annotate classifier type 
        classifier_type = tracking_region.protocol.classifier.type
        if classifier_type != 'empty':
            classifier_name = tracking_region.protocol.classifier.classifier_param['display_name']
            display_text[arena]['classifier']['text'] = classifier_name
            text_size = cv2.getTextSize(classifier_name, self.font, self.fontsize, self.fontthickness)
            display_text[arena]['classifier']['font'] = self.font
            display_text[arena]['classifier']['posX'] = x0 + self.text_offset
            display_text[arena]['classifier']['posY'] = y1 - self.text_offset
            display_text[arena]['classifier']['color'] = color
        
            # Annotate fly genotype
            genotype_display_name = tracking_region.param['default_param']['fly']['display_name'][genotype_name]
            display_text[arena]['fly']['text'] = genotype_display_name
            text_size = cv2.getTextSize(genotype_display_name, self.font, self.fontsize, self.fontthickness)
            display_text[arena]['fly']['font'] = self.font
            display_text[arena]['fly']['posX'] = x1 - text_size[0][0] - self.text_offset
            display_text[arena]['fly']['posY'] = y0 + text_size[0][1] + self.text_offset
            display_text[arena]['fly']['color'] = color
            
            # Annotate region number
            display_text[arena]['region']['text'] = str(arena)
            text_size = cv2.getTextSize(str(arena), self.font, self.fontsize, self.fontthickness)
            display_text[arena]['region']['font'] = self.font
            display_text[arena]['region']['posX'] = x0 + self.text_offset
            display_text[arena]['region']['posY'] = y0 + text_size[0][1] + self.text_offset
            display_text[arena]['region']['color'] = color
    
    def draw_bounding_box(self, image, tracking_region, color):
        classifier_type = tracking_region.protocol.classifier.type
        if classifier_type != 'empty':
            x0, y0 = tracking_region.upper_left
            x1, y1 = tracking_region.lower_right
            cv2.rectangle(image, (x0,y0), (x1,y1), color, self.bounding_box_linewidth)                
    
    def draw_display_text(self, image, elapsed_time, trial_scheduler, display_text):

        # elapsed time
        t_min = int(np.floor(elapsed_time/60.0))
        t_sec = int(60.0*(elapsed_time/60.0 - t_min))
        text = '{}:{:02d}'.format(t_min, t_sec)
        color = self.yellow
        cv2.putText(image, text, (50,100), self.font, self.fontsize_large, color, self.fontthickness_thick)
        

        # trial state
        text = trial_scheduler.state
        if trial_scheduler.state == 'enabled':
            color = self.red
        else:
            color = self.gray
        display_font = self.fontsize_large
        cv2.putText(image, text, (50,150), self.font, self.fontsize_large, color, self.fontthickness_thick)
        # annotate regions
        for arena in self.display_text:
            for item in self.display_text[arena]:
                try:
                    posX = display_text[arena][item]['posX']
                    posY = display_text[arena][item]['posY']
                    text = display_text[arena][item]['text']
                    color = display_text[arena][item]['color']
                    font = display_text[arena][item]['font']
                    cv2.putText(image, text, (posX,posY), self.font, self.fontsize, color, self.fontthickness)
                except:
                    pass
        return image

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
        # Determine color based on LED
        led_scheduler_type = tracking_region.protocol.led_scheduler.type
        if led_scheduler_type == 'instant' or led_scheduler_type == 'pulse':
            color = self.classifier_color
        else:
            color = self.classifier_off_color
        offset = 6
        if classifier_type == 'center':
            cx = tracking_region.param['center']['cx']
            cy = tracking_region.param['center']['cy']
            if 'radius' in classifier_param:
                radius = classifier_param['radius']
                cv2.circle(image, (cx,cy), radius, color,self.classifier_thickness)
            elif 'height' in classifier_param:
                height = classifier_param['height']
                width = classifier_param['width']
                bottom_left = (cx-width/2,cy+height/2)
                top_right = (cx+width/2,cy-height/2)
                cv2.rectangle(image, bottom_left, top_right, color,self.classifier_thickness)
        elif classifier_type == 'tunnels':
            cx = tracking_region.param['center']['cx']
            cy = tracking_region.param['center']['cy']
            radius = classifier_param['radius']
            outer_radius = classifier_param['outer_radius']
            tunnels = classifier_param['tunnels']
#            if 'left' in tunnels:
#                x = cx - radius
#                cv2.line(image, (x,cy-5), (x,cy-12), color, self.classifier_thickness)
#                cv2.line(image, (x,cy+5), (x,cy+12), color, self.classifier_thickness)
#            if 'right' in tunnels:
#                x = cx + radius
#                cv2.line(image, (x,cy-5), (x,cy-12), color, self.classifier_thickness)
#                cv2.line(image, (x,cy+5), (x,cy+12), color, self.classifier_thickness)
#            if 'top' in tunnels:
#                y = cy - radius
#                cv2.line(image, (cx-5,y), (cx-12,y), color, self.classifier_thickness)
#                cv2.line(image, (cx+5,y), (cx+12,y), color, self.classifier_thickness)
#            if 'bottom' in tunnels:
#                y = cy + radius
#                cv2.line(image, (cx-5,y), (cx-12,y), color, self.classifier_thickness)
#                cv2.line(image, (cx+5,y), (cx+12,y), color, self.classifier_thickness)
                
            if 'top_left' in tunnels:
                start_angle = -150
                end_angle = -120
                cv2.ellipse(image, (cx,cy), (radius, radius), 0, start_angle, end_angle, color, self.classifier_thickness)
                cv2.ellipse(image, (cx,cy), (outer_radius, outer_radius), 0, start_angle, end_angle, color, self.classifier_thickness)
            if 'top_right' in tunnels:
                start_angle = -60
                end_angle = -30
                cv2.ellipse(image, (cx,cy), (radius, radius), 0, start_angle, end_angle, color, self.classifier_thickness)
                cv2.ellipse(image, (cx,cy), (outer_radius, outer_radius), 0, start_angle, end_angle, color, self.classifier_thickness)
            if 'bottom_left' in tunnels:
                start_angle = 120
                end_angle = 150
                cv2.ellipse(image, (cx,cy), (radius, radius), 0, start_angle, end_angle, color, self.classifier_thickness)
                cv2.ellipse(image, (cx,cy), (outer_radius, outer_radius), 0, start_angle, end_angle, color, self.classifier_thickness)
            if 'bottom_right' in tunnels:
                start_angle = 30
                end_angle = 60
                cv2.ellipse(image, (cx,cy), (radius, radius), 0, start_angle, end_angle, color, self.classifier_thickness)
                cv2.ellipse(image, (cx,cy), (outer_radius, outer_radius), 0, start_angle, end_angle, color, self.classifier_thickness)
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
