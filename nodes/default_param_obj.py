from __future__ import print_function

class DefaultParamObj(object):

    def __init__(self,param):
        self.param = dict(param)

    @property
    def global_default_param(self):
        return {}

    @property
    def type_specific_param(self):
        return {}

    @property
    def instance_specific_param(self):
        return {}

    def get_param(self):
        global_param = self.global_default_param
        type_specific_param = self.type_specific_param
        instance_specific_param = self.instance_specific_param
        param = dict(global_param)
        param.update(type_specific_param)
        param.update(instance_specific_param)
        return param


