
class LedScheduler(object):

    def __init__(self,param):
        self.param = param
        self.scheduler_param = self.param['protocol'][1][1]

        self.led_value = 0
        self.led_state = False

    def turn_on_led(self):
        print('region: {} led on', self.param['index'])

    def turn_off_led(self):
        print('region: {} led off', self.param['index'])

    def update(self, t, current_object, classifier_state):
        pass



