import math
import numpy as np
from classifier import Classifier

class TunnelsInvertedClassifier(Classifier):

    tunnel_to_sign_dict = {
            'top_right'     : ( 1, -1),
            'top_left'      : (-1, -1),
            'bottom_right'  : ( 1,  1),
            'bottom_left'   : (-1,  1),
            }

    def __init__(self,param):
        super(TunnelsInvertedClassifier,self).__init__(param)

    def update(self, t, obj_dict):
        current_object = obj_dict['fly']
        if current_object is not None:
            x = current_object.position.x
            y = current_object.position.y
            cx = self.param['center']['cx']
            cy = self.param['center']['cy']
            radius = self.classifier_param['radius']
            outer_radius = self.classifier_param['outer_radius']
            tunnels = self.classifier_param['tunnels']
            tunnel_signs = [self.tunnel_to_sign_dict[k] for k in tunnels]
            radial_dist = math.sqrt((cx-x)**2 + (cy-y)**2)

            if radial_dist > radius and radial_dist < outer_radius:
               sx = np.sign(x-cx)
               sy = np.sign(y-cy)
               if (sx,sy) in tunnel_signs:
                   self.state = False
            else:
                self.state = True
