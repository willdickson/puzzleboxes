import math
from classifier import Classifier

class EmptyClassifier(Classifier):

    def __init__(self,param):
        super(EmptyClassifier,self).__init__(param)

    def update(self,t,obj_dict):
        self.state = False





    
