from __future__ import print_function
import time
from led_controller import LedController as LedControllerBase
import Phidgets.Devices.LED

class LedController(LedControllerBase):

    def __init__(self):
        super(LedController,self).__init__()
        self.device = Phidgets.Devices.LED.LED()
        self.device.openPhidget()
        self.device.waitForAttach(2000)

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


