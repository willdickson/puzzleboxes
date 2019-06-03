from __future__ import print_function 

class Classifier(object):

    def __init__(self,param):
        self.param = param
        self.state = False
        self.extract_classifier_param()

    def extract_classifier_param(self):
        self.classifier_type = self.param['protocol'][0][0]

        global_param = {}
        try:
            global_param = self.param['default_param']['classifier']['global']
        except KeyError:
            pass

        type_specific_param = {}
        try:
            type_specific_param = self.param['default_param']['classifier']['by_type'][self.classifier_type]
        except KeyError:
            pass

        instance_specific_param = self.param['protocol'][0][1]

        self.classifier_param = global_param
        self.classifier_param.update(type_specific_param)
        self.classifier_param.update(instance_specific_param)

    def update(self,t,current_object):
        pass
