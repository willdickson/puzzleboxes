from led_scheduler import LedScheduler

class EmptyLedScheduler(LedScheduler):

    def __init__(self,param,devices):
        super(EmptyLedScheduler,self).__init__(param,devices)

    def update(self, t, current_object, classifier_state):
        self.led_off()
