from __future__ import print_function
from led_scheduler import LedScheduler

class PulseLedScheduler(LedScheduler):

    def __init__(self,param,devices):
        super(PulseLedScheduler,self).__init__(param,devices)
        self.last_on_t  = 0.0

    def update(self, t, current_object, classifier_state):
        if self.state:
            if (t - self.last_on_t) > self.led_scheduler_param['on_t']:
                self.led_off()
        else:
            if classifier_state:
                if (t - self.last_on_t) > (self.led_scheduler_param['on_t'] + self.led_scheduler_param['off_t']):
                    self.last_on_t = t
                    self.led_on()   

