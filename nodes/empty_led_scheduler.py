from led_scheduler import LedScheduler

class EmptyLedScheduler(LedScheduler):

    def __init__(self,param,devices):
        super(EmptyLedScheduler,self).__init__(param,devices)

    def update(self, t, obj_dict, classifier_state):
        self.led_off()
