from __future__ import print_function
from led_scheduler import LedScheduler

class PulseLedScheduler(LedScheduler):

    def __init__(self,param,devices):
        super(PulseLedScheduler,self).__init__(param,devices)
        self.roi_to_last_on_t = {}

    def update(self, t, obj_dict, classifier_state):

        if self.state:
            off_test = True 
            for roi_ind, last_on_t in self.roi_to_last_on_t.items():
                off_test &= (t - last_on_t) > self.led_scheduler_param['on_t']
            if off_test:
                self.led_off()

        if classifier_state:
            try:
                last_on_t = self.roi_to_last_on_t[classifier_state]
            except KeyError:
                last_on_t = None

            if last_on_t is not None:
                on_test = (t - last_on_t) > (self.led_scheduler_param['on_t'] + self.led_scheduler_param['off_t'])
            else:
                on_test = True

            if on_test:
                self.roi_to_last_on_t[classifier_state] = t
                self.led_on()   
