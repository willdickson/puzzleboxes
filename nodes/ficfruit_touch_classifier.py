from __future__ import print_function
import math
import numpy as np
from classifier import Classifier

class FicFruitTouchClassifier(Classifier):

    def __init__(self,param):
        super(FicFruitTouchClassifier,self).__init__(param)
        self.count = 0

    def update(self,t,obj_dict):

        if not obj_dict:
            self.state = False
            return

        fly = obj_dict['fly']
        ball = obj_dict['ball']
        fly_pos = np.array(fly.centroid)
        ball_pos = np.array(ball.centroid)
        dist = np.sqrt(((fly_pos - ball_pos)**2).sum())

        if dist <= self.classifier_param['distance']:
            self.count += 1
            if self.count >= self.classifier_param['touch_count']:
                self.state = True
        else:
            self.count = 0
            self.state = False




    
