from led_scheduler import LedScheduler

class InstantLedScheduler(LedScheduler):

    def __init__(self,param):
        super(InstantLedScheduler,self).__init__(param)

    def update(self, t, current_object, classifier_state):
        if classifier_state:
            self.led_on()
        else:
            self.led_off()
