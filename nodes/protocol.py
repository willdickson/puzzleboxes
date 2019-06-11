from empty_classifier import EmptyClassifier
from center_classifier import CenterClassifier
from empty_led_scheduler import EmptyLedScheduler
from pulse_led_scheduler import PulseLedScheduler
from instant_led_scheduler import InstantLedScheduler

class Protocol(object):

    ClassifierTable = {
            'empty'  :  EmptyClassifier,
            'center' :  CenterClassifier,
             
            }

    LedSchedulerTable = {
            'empty'   : EmptyLedScheduler,
            'pulse'   : PulseLedScheduler,
            'instant' : InstantLedScheduler,
            }

    def __init__(self,param):
        self.param = param
        self.create_classifier()
        self.create_led_scheduler()

    def led_enable(self,value):
        self.led_scheduler.enabled(value)

    def create_classifier(self):
        #classifier_type = self.param['protocol'][0][0]
        classifier_type = self.param['protocol']['classifier']['type']
        self.classifier =  self.ClassifierTable[classifier_type](self.param)

    def create_led_scheduler(self):
        #led_scheduler_type = self.param['protocol'][1][0]
        led_scheduler_type = self.param['protocol']['led_policy']['type']
        self.led_scheduler = self.LedSchedulerTable[led_scheduler_type](self.param)

    def update(self,t,current_object):
        self.classifier.update(t,current_object)
        self.led_scheduler.update(t,current_object,self.classifier.state)
