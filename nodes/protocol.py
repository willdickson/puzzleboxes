from __future__ import print_function

from empty_classifier import EmptyClassifier
from center_classifier import CenterClassifier
from roi_classifier import ROIClassifier
from rois_classifier import ROIsClassifier
from roi_revisit_classifier import ROIRevisitClassifier
from tunnels_classifier import TunnelsClassifier
from ficfruit_touch_classifier import FicFruitTouchClassifier
from ficfruit_revisit_classifier import FicFruitRevisitClassifier


from empty_led_scheduler import EmptyLedScheduler
from pulse_led_scheduler import PulseLedScheduler
from single_pulse_led_scheduler import SinglePulseLedScheduler
from instant_led_scheduler import InstantLedScheduler
from on_led_scheduler import OnLedScheduler

class Protocol(object):

    ClassifierTable = {
            'empty'            : EmptyClassifier,
            'center'           : CenterClassifier,
            'roi'              : ROIClassifier,
            'rois'             : ROIsClassifier,
            'roi_revisit'      : ROIRevisitClassifier,
            'tunnels'          : TunnelsClassifier,
            'ficfruit_touch'   : FicFruitTouchClassifier, 
            'ficfruit_revisit' : FicFruitRevisitClassifier,
            }

    LedSchedulerTable = {
            'empty'        : EmptyLedScheduler,
            'pulse'        : PulseLedScheduler,
            'single_pulse' : SinglePulseLedScheduler,
            'instant'      : InstantLedScheduler,
            'always_on'    : OnLedScheduler,
            }

    def __init__(self,param, devices):
        self.param = dict(param)
        self.devices = devices
        self.create_classifier()
        self.create_led_scheduler()

    def led_enable(self,value):
        self.led_scheduler.enabled(value)

    def create_classifier(self):
        classifier_type = self.param['protocol']['classifier']['type']
        self.classifier =  self.ClassifierTable[classifier_type](self.param)

    def create_led_scheduler(self):
        led_scheduler_type = self.param['protocol']['led_policy']['type']
        self.led_scheduler = self.LedSchedulerTable[led_scheduler_type](self.param,self.devices)

    def update(self,t,obj_dict,led_enabled):
        self.classifier.update(t, obj_dict)
        if led_enabled:
            self.led_scheduler.update(t, obj_dict, self.classifier.state)
        else:
            self.led_scheduler.led_off()
