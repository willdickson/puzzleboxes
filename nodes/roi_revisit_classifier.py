import math
from classifier import Classifier

class ROIRevisitClassifier(Classifier):

    def __init__(self,param):
        super(ROIRevisitClassifier,self).__init__(param)
        self.last_state = False

    def update(self,t,obj_dict):
        current_object = obj_dict['fly']
        if current_object is not None:
            x = current_object.position.x
            y = current_object.position.y
            cx = self.param['center']['cx']+self.classifier_param['x_pos']
            cy = self.param['center']['cy']+self.classifier_param['y_pos']
            dist = math.sqrt((cx-x)**2 + (cy-y)**2)

            # Select radius based on previous state for hysteresis
            if self.last_state:
                radius = self.classifier_param['outer_radius']
            else:
                radius = self.classifier_param['inner_radius']

            if dist < radius: 
                self.state = True
            else:
                self.state = False
        else:
            self.state = False

        self.last_state = self.state





    
