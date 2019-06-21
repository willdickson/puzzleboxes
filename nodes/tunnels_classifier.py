import math
from classifier import Classifier

class TunnelsClassifier(Classifier):

    def __init__(self,param):
        super(TunnelsClassifier,self).__init__(param)

    def update(self,t,current_object):
        if current_object is not None:
            x = current_object.position.x
            y = current_object.position.y
            cx = self.param['center']['cx']
            cy = self.param['center']['cy']
            radius = self.classifier_param['radius']
            tunnels = self.classifier_param['tunnels']
            radial_dist = math.sqrt((cx-x)**2 + (cy-y)**2)
            # first check if fly is in a tunnel
            if radial_dist > radius:
                # next check if fly is within classified tunnels
                if 'left' in tunnels and x-cx < -1.0*radius:
                    self.state = True
                elif 'right' in tunnels and x-cx > radius:
                    self.state = True
                elif 'top' in tunnels and y-cy < -1.0*radius:
                    self.state = True
                elif 'bottom' in tunnels and y-cy > radius:
                    self.state = True
                else:
                    self.state = False
            else:
                self.state = False
        else:
            self.state = False





    
