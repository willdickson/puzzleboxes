from __future__  import print_function
import yaml 
from default_param_obj import DefaultParamObj

class LedScheduler(DefaultParamObj):

    def __init__(self,param):
        self.param = param
        self.led_scheduler_param = self.get_param() 
        self.value = 0
        self.state = False
        self.is_enabled = False
        self.led_off(force=True)

    @property
    def global_default_param(self):
        global_param = {}
        try:
            global_param = self.param['default_param']['led_scheduler']['global']
        except KeyError:
            pass
        return global_param


    @property
    def type_specific_param(self):
        type_specific_param = {}
        try:
            type_specific_param = self.param['default_param']['led_scheduler']['by_type'][self.type]
        except KeyError:
            pass
        return type_specific_param
    
    @property
    def instance_specific_param(self):
        #return self.param['protocol'][1][1]
        return yaml.load(self.param['protocol']['led_policy']['param'])

    @property
    def type(self):
        #return self.param['protocol'][1][0]
        return self.param['protocol']['led_policy']['type']


    def enabled(self,value):
        if self.state == True and value == False:
            self.led_off()
        self.enabled = value

    def led_on(self):
        if not self.is_enabled:
            return
        if not self.state:
            self.state  = True
            #print('region: {} led on', self.param['index'])

    def led_off(self,force=False):
        if self.state or force:
            self.state = False
            #print('region: {} led off', self.param['index'])

    def update(self, t, current_object, classifier_state):
        pass



