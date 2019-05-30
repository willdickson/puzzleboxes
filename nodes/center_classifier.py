from classifier import Classifier

class CenterClassifier(Classifier):

    def __init__(self,param):
        super(Classifier,self).__init__(param)

    def update(self,t,current_object):
        if 'radius' in self.param['protocol']['classifier']:
            pass
        else:
            pass



    
