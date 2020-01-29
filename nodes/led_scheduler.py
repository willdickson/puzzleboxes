from __future__  import print_function
import yaml 
from default_param_obj import DefaultParamObj
import rospy

class LedScheduler(DefaultParamObj):

    def __init__(self,param,devices):
        super(LedScheduler,self). __init__(param)
        self.devices = devices
        self.led_scheduler_param = self.get_param() 
        self.value = 0
        self.state = False
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
        return yaml.load(self.param['protocol']['led_policy']['param'])

    @property
    def type(self):
        return self.param['protocol']['led_policy']['type']

    @property
    def led_numbers(self):
        led_numbers = self.param['protocol']['led_numbers']
        if led_numbers == 'all':
            return [n+1 for n in self.param['ledmap']]
        else:
            return yaml.load(led_numbers)

    def enabled(self,value):
        if self.state == True and value == False:
            self.led_off()
        self.enabled = value

    def led_on(self):
        if not self.state:
            self.state  = True
            for lednum in self.led_numbers:
                ledind = lednum-1
                ledpin = self.param['ledmap'][ledind]
                brightness = self.led_scheduler_param['brightness']
                self.devices['led_controller'].set_led(ledpin,brightness)

    def led_off(self,force=False):
        if self.state or force:
            self.state = False
            for lednum in self.led_numbers:
                ledind = lednum - 1
                ledpin = self.param['ledmap'][ledind]
                self.devices['led_controller'].set_led(ledpin,0)

    def update(self, t, current_object, classifier_state):
        pass



