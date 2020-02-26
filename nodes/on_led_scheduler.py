from led_scheduler import LedScheduler

class OnLedScheduler(LedScheduler):

    def __init__(self,param,devices):
        super(OnLedScheduler,self).__init__(param,devices)

    def update(self, t, obj_dict, classifier_state):
        self.led_on()
