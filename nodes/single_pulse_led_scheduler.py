from __future__ import print_function
from led_scheduler import LedScheduler

class SinglePulseLedScheduler(LedScheduler):

    def __init__(self,param,devices):
        super(SinglePulseLedScheduler,self).__init__(param,devices)
        self.on_t = 0.0
        self.pulsed = False

    def update(self, t, obj_dict, classifier_state):
        if self.state:
            on_duration = self.led_scheduler_param['on_t']
            if (t - self.on_t) > on_duration:
                self.led_off()
        else:
            if classifier_state:
                if not self.pulsed:
                    self.pulsed = True
                    self.on_t = t
                    self.led_on()   
            else:
                self.pulsed = False

