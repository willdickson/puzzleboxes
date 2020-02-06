from __future__ import print_function
import math
import numpy as np
from classifier import Classifier

class FicFruitRevisitClassifier(Classifier):

    def __init__(self,param):
        super(FicFruitRevisitClassifier,self).__init__(param)
        self.count = 0
        self.last_state = False

    def update(self,t,obj_dict):

        if not obj_dict:
            self.state = False
            return

        fly = obj_dict['fly']
        ball = obj_dict['ball']
        fly_pos = np.array([fly['centroidY'], fly['centroidX']])
        ball_pos = np.array([ball['centroidY'], ball['centroidX']])
        dist = np.sqrt(((fly_pos - ball_pos)**2).sum())

        # Select radius based on previous state for hysteresis
        if self.last_state:
            radius = self.classifier_param['outer_radius']
        else:
            radius = self.classifier_param['inner_radius']

        if dist <= radius: 
            self.count += 1
            if self.count >= self.classifier_param['touch_count']:
                self.state = True
        else:
            self.count = 0
            self.state = False

        self.last_state = self.state





    
