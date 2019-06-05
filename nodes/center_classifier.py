import math
from classifier import Classifier

class CenterClassifier(Classifier):

    def __init__(self,param):
        super(CenterClassifier,self).__init__(param)

    def update(self,t,current_object):
        if current_object is not None:
            x = current_object.position.x
            y = current_object.position.y
            cx = self.param['center']['cx']
            cy = self.param['center']['cy']
            if 'radius' in self.classifier_param:
                dist = math.sqrt((cx-x)**2 + (cy-y)**2)
                if dist < self.classifier_param['radius']:
                    self.state = True
                else:
                    self.state = False
            elif 'height' in self.classifier_param:
                h = self.classifier_param['height']
                w = self.classifier_param['width']
                test_w = abs((cx-x)) < 0.5*w
                test_h = abs((cy-y)) < 0.5*h
                if abs((cx-x)) < 0.5*w and abs((cy-y)) < 0.5*h:
                    self.state = True
                else:
                    self.state = False
                #if self.param['index'] == 1:
                #    print('dx: {:0.1f} dy: {:0.1f}, tw: {}, th: {}, state: {}'.format(abs(cx-x), abs(cy-y), test_w, test_h, self.state))
        else:
            self.state = False





    
