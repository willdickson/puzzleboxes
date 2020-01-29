from __future__ import print_function
import os
import cv2
import numpy as np
import yaml
import rospy
import Queue

class RegionVisualizer(object):

    def __init__(self,param,data_queue=None):

        self.param = param
        self.data_queue = data_queue
        self.done = False
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
                
        self.object_radius = 15 
        self.object_linewidth_thin = 2
        self.object_linewidth_thick = 3
        self.classifier_color = self.yellow
        self.classifier_off_color = self.white
        self.classifier_thickness = 2
        self.bounding_box_linewidth = 2

        self.find_centers = False
        self.centers_count = 0

        self.font = cv2.FONT_HERSHEY_SIMPLEX

        self.fontsize = 0.5
        self.fontsize_large = 0.75 
        self.fontthickness = 1
        self.fontthickness_thick = 2 
        self.text_offset = 3 
        

    def run(self):
        while not self.done:
            data = None
            while self.data_queue.qsize() > 0:
                data = self.data_queue.get()
            if data is not None:
                self.update(data)
    

    def update(self, data): 
        elapsed_time = data['elapsed_time'] 
        image = data['bgr_image']
        trial_scheduler = data['trial_scheduler']
        tracking_region_list = data['tracking_region_list']

        if image is None:
            return

        if self.is_first_image: 
            # Setup opencv plotting windows
            cv2.namedWindow(self.window_name,cv2.WINDOW_AUTOSIZE)
            #cv2.namedWindow(self.window_name,cv2.WINDOW_NORMAL)
            #cv2.moveWindow(self.window_name, 100, 100)
            #cv2.resizeWindow(self.window_name, 800,600) 
            self.is_first_image = False 
            #if self.find_centers:
                #cv2.namedWindow('centers_image',cv2.WINDOW_NORMAL)
                #cv2.moveWindow('centers_image', 120, 120)
                #cv2.resizeWindow('centers_image', 800,600)
        
        self.display_text = {}
                
        for tracking_region in tracking_region_list:
            color = self.determine_region_color(tracking_region)
            self.annotate_region(image,tracking_region, self.display_text, color)
            self.draw_bounding_box(image,tracking_region, color)
            self.draw_classifier(image,tracking_region)
            self.draw_object(image, tracking_region)

        self.draw_display_text(image, elapsed_time, trial_scheduler, self.display_text)
	
        if self.param['use_compressed_images']:
            n,m,c = image.shape
            ns, ms = int(n*self.param['display_scale']), int(m*self.param['display_scale'])
            #image = cv2.resize(image,(ms,ns),0,0,cv2.INTER_AREA)
            image = cv2.resize(image,(ms,ns),0,0,cv2.INTER_NEAREST)
            #image = image[:ns,:ms]
        
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

        classifier_state = tracking_region.protocol.classifier.state
        led_state = tracking_region.protocol.led_scheduler.state

        if hasattr(tracking_region, 'obj'):
            if tracking_region.obj is not None:
                x = int(tracking_region.obj.position.x)
                y = int(tracking_region.obj.position.y)
                object_color = self.states_to_object_color[(classifier_state,led_state)]
                if led_state or classifier_state:
                    object_linewidth = self.object_linewidth_thick
                else:
                    object_linewidth = self.object_linewidth_thin
                cv2.circle(image, (x,y), self.object_radius, object_color, object_linewidth)

        if hasattr(tracking_region, 'obj_dict'):
            if tracking_region.obj_dict:
                fly_color = self.states_to_object_color[(classifier_state,led_state)]
                if led_state or classifier_state:
                    object_linewidth = self.object_linewidth_thick
                else:
                    object_linewidth = self.object_linewidth_thin


                if tracking_region.obj_dict['ball'] is not None:
                    ball = tracking_region.obj_dict['ball']
                    ball_x = int(ball['centroidX']) + tracking_region.x0
                    ball_y = int(ball['centroidY']) + tracking_region.y0
                    cv2.circle(image, (ball_x,ball_y), self.object_radius, (0,0,0), object_linewidth)

                if tracking_region.obj_dict['fly'] is not None:
                    fly = tracking_region.obj_dict['fly']
                    fly_x = int(fly['centroidX']) + tracking_region.x0
                    fly_y = int(fly['centroidY']) + tracking_region.y0
                    cv2.circle(image, (fly_x,fly_y), self.object_radius, fly_color, object_linewidth)
        
    def draw_classifier(self, image, tracking_region):
        classifier_type = tracking_region.protocol.classifier.type
        classifier_param = tracking_region.protocol.classifier.classifier_param
        # Determine color based on LED
        led_scheduler_type = tracking_region.protocol.led_scheduler.type
        if led_scheduler_type == 'instant' or led_scheduler_type == 'pulse':
            color = self.classifier_color
        else:
            color = self.classifier_off_color
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
        elif classifier_type == 'roi':
            cx = tracking_region.param['center']['cx']+classifier_param['x_pos']
            cy = tracking_region.param['center']['cy']+classifier_param['y_pos']
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






