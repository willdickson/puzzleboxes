
class LedController(object):

    LedMaxValue = 100
    LedMinValue = 0

    def __init__(self):
        self.num_led = 35

    def set_led(self,index,value):
        value = clamp(value,self.LedMinValue,self.LedMaxValue)
        self.device_set_led(index,value)

    def set_all_led(self,value):
        for i in range(self.num_led):
            self.set_led(i,value)

    def device_set_led(self,index,value):
        pass

def clamp(x,min_value,max_value):
    x = max(x,min_value)
    x = min(x,max_value)
    return x


