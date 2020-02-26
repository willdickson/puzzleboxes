import math
from classifier import Classifier
import rospy

class ROIsClassifier(Classifier):

    def __init__(self,param):
        super(ROIsClassifier,self).__init__(param)

    def update(self,t,obj_dict):
        current_object = obj_dict['fly']
        if current_object is not None:
            x = current_object.position.x
            y = current_object.position.y
            positions = zip(self.classifier_param['x_pos'], self.classifier_param['y_pos'])
            for position in positions:
                cx = self.param['center']['cx']+position[0]
                cy = self.param['center']['cy']+position[1]
                if 'radius' in self.classifier_param:
                    dist = math.sqrt((cx-x)**2 + (cy-y)**2)
                    if dist < self.classifier_param['radius']:
                        self.state = position 
                        break
                    else:
                        self.state = False 
                elif 'height' in self.classifier_param:
                    h = self.classifier_param['height']
                    w = self.classifier_param['width']
                    test_w = abs((cx-x)) < 0.5*w
                    test_h = abs((cy-y)) < 0.5*h
                    if abs((cx-x)) < 0.5*w and abs((cy-y)) < 0.5*h:
                        self.state = position 
                    else:
                        self.state = False
        else:
            self.state = False





    
