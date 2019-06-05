from center_classifier import CenterClassifier
from pulse_led_scheduler import PulseLedScheduler
from instant_led_scheduler import InstantLedScheduler

class Protocol(object):

    ClassifierTable = {
            'center' :  CenterClassifier, 
            }

    LedSchedulerTable = {
            'pulse'   : PulseLedScheduler,
            'instant' : InstantLedScheduler,
            }

    def __init__(self,param):
        self.param = param
        self.create_classifier()
        self.create_led_scheduler()

    def create_classifier(self):
        classifier_type = self.param['protocol'][0][0]
        self.classifier =  self.ClassifierTable[classifier_type](self.param)

    def create_led_scheduler(self):
        led_scheduler_type = self.param['protocol'][1][0]
        self.led_scheduler = self.LedSchedulerTable[led_scheduler_type](self.param)

    def update(self,t,current_object):
        self.classifier.update(t,current_object)
        self.led_scheduler.update(t,current_object,self.classifier.state)
