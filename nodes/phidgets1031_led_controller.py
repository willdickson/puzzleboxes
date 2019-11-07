from __future__ import print_function

import time
from led_controller import LedController as LedControllerBase

import Phidgets.Devices.LED
from Phidgets.Devices.LED import LED, LEDCurrentLimit, LEDVoltage

class LedController(LedControllerBase):

    LimitToString = {
            LEDCurrentLimit.CURRENT_LIMIT_20mA: '20mA',
            LEDCurrentLimit.CURRENT_LIMIT_40mA: '40mA',
            LEDCurrentLimit.CURRENT_LIMIT_60mA: '60mA',
            LEDCurrentLimit.CURRENT_LIMIT_80mA: '80mA',
            }
    StringToLimit = {v:k for k,v in LimitToString.items()}

    def __init__(self):
        super(LedController,self).__init__()
        self.device = Phidgets.Devices.LED.LED()
        self.device.openPhidget()
        self.device.waitForAttach(2000)

    def set_current_limit(self,limit_str):
        limit_val = self.StringToLimit[limit_str]
        self.device.setCurrentLimit(limit_val)

    def get_current_limit(self):
        limit_val = self.device.getCurrentLimit()
        limit_str = self.LimitToString[limit_val]
        return limit_str

    def device_set_led(self,index,value):
        self.device.setBrightness(index,value)
        time.sleep(0.001)
    
# Testing 
# -------------------------------------------------------------
if __name__ == '__main__':

    ctlr = LedController()
    for i in range(10):
        ctlr.set_all_led(50)
        time.sleep(1.0)
        ctlr.set_all_led(0)
        time.sleep(1.0)


