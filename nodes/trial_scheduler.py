from __future__ import print_function
import rospy

class TrialScheduler(object):

    def __init__(self,param):
        self.param = param
        self.check_states()
        self.set_state(0,0.0)
        self.done = False

    def update(self, t):
        if t >= self.start_t + self.duration:
            ok = self.set_state(self.index + 1,t)
            if not ok:
                self.done = True

    def set_state(self,index,t):
        if index < len(self.param):
            self.index = index
            self.state = self.param[index]['state'] 
            self.start_t = t 
            return True
        else:
            self.state = 'disabled'
            return False

    def check_states(self):
        for state_dict in self.param:
            state = state_dict['state']
            assert(state in ('enabled', 'disabled'), 'uknown trial state{}'.format(state))

    @property
    def duration(self):
        return min_to_sec(self.param[self.index]['duration'])

    @property
    def led_enabled(self):
        return self.state == 'enabled'

def min_to_sec(t_min):
    return 60.0*t_min


