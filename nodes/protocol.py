
class Protocol(object):

    def __init__(self,param):
        self.param = param
        self.create_classifier()
        self.create_led_scheduler()

    def create_classifier(self):
        pass

    def create_led_scheduler(self):
        pass

    def update(self,t,current_object):
        pass
