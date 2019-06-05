from __future__ import print_function 
from default_param_obj import DefaultParamObj

class Classifier(DefaultParamObj):

    def __init__(self,param):
        super(Classifier,self). __init__(param)
        self.classifier_param = self.get_param()
        self.state = False

    @property
    def global_default_param(self):
        global_param = {}
        try:
            global_param = self.param['default_param']['classifier']['global']
        except KeyError:
            pass
        return global_param

    @property
    def type_specific_param(self):
        type_specific_param = {}
        try:
            type_specific_param = self.param['default_param']['classifier']['by_type'][self.type]
        except KeyError:
            pass
        return type_specific_param
    
    @property
    def instance_specific_param(self):
        return self.param['protocol'][0][1]

    @property
    def type(self):
        return self.param['protocol'][0][0]


    def update(self,t,current_object):
        pass
