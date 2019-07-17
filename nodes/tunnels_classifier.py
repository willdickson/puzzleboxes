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
            outer_radius = self.classifier_param['outer_radius']
            tunnels = self.classifier_param['tunnels']
            radial_dist = math.sqrt((cx-x)**2 + (cy-y)**2)
            # first check if fly is in a tunnel
            if radial_dist > radius:
                # next check if fly is within classified tunnels
                left = False
                right = False
                top = False
                bottom = False
                if -1.0*outer_radius < (x-cx) < -1.0*radius:
                    left = True
                if outer_radius > (x-cx) > radius:
                    right = True
                if -1.0*outer_radius < (y-cy) < -1.0*radius:
                    top = True
                if outer_radius > (y-cy) > radius:
                    bottom = True
                    
                self.state = False
                if 'left' in tunnels and '_left' not in tunnels and left:
                    self.state = True
                if 'right' in tunnels and '_right' not in tunnels and right:
                    self.state = True    
                if 'top' in tunnels and 'top_' not in tunnels and top:
                    self.state = True
                if 'bottom' in tunnels and 'bottom_' not in tunnels and bottom:
                    self.state = True
                if 'top_left' in tunnels and left and top:
                    self.state = True
                if 'top_right' in tunnels and right and top:
                    self.state = True        
                if 'bottom_left' in tunnels and left and bottom:
                    self.state = True
                if 'bottom_right' in tunnels and right and bottom:
                    self.state = True
                   
            else:
                self.state = False
        else:
            self.state = False
