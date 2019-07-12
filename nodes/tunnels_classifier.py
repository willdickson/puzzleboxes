import math
from classifier import Classifier
import rospy

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
#                if -1.0*outer_radius < (x-cx) < -1.0*radius:
                if -1.0*outer_radius < (x-cx) and (x-cx) < -1.0*radius:    
                    left = True
                    self.state = True
                    posit=[str(x),str(y)]
                    rospy.logwarn(posit)
                if outer_radius > (x-cx) > radius:
                    right = True
                    self.state = True
                    posit=[str(x),str(y)]
                    rospy.logwarn(posit)
                if -1.0*outer_radius < (y-cy) < -1.0*radius:
                    top = True
                    self.state = True
                    posit=[str(x),str(y)]
                    rospy.logwarn(posit)
                if outer_radius > (y-cy) > radius:
                    bottom = True
                    self.state = True
                    posit=[str(x),str(y)]
                    rospy.logwarn(posit)
                
                
#                if 'left' in tunnels and -1.0*outer_radius < x-cx < -1.0*radius:
#                    self.state = True
#                elif 'right' in tunnels and outer_radius > x-cx > radius:
#                    self.state = True
#                elif 'top' in tunnels and -1.0*outer_radius < y-cy < -1.0*radius:
#                    self.state = True
#                elif 'bottom' in tunnels and outer_radius > y-cy > radius:
#                    self.state = True
#                elif 'top_left' in tunnels and -1.0*outer_radius < x-cx < -1.0*radius and -1.0*outer_radius < y-cy < -1.0*radius:
#                    self.state = True
#                elif 'top_right' in tunnels and outer_radius > x-cx > radius and -1.0*outer_radius < y-cy < -1.0*radius:
#                    self.state = True
#                elif 'bottom_left' in tunnels and -1.0*outer_radius < x-cx < -1.0*radius and outer_radius > y-cy > radius:
#                    self.state = True
#                elif 'bottom_right' in tunnels and outer_radius > x-cx > radius and outer_radius > y-cy > radius:
#                    self.state = True    
                else:
                    self.state = False
            else:
                self.state = False
        else:
            self.state = False





    
