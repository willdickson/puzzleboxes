
class RectRegion(object):

    def __init__(self, param):
        self.param = dict(param)

    @property
    def x0(self):
        return self.param['roi']['x0']

    @property
    def x1(self):
        return self.param['roi']['x1']

    @property
    def y0(self):
        return self.param['roi']['y0']

    @property
    def y1(self):
        return self.param['roi']['y1']

    def contains(self, obj):
        x,y = obj.position.x, obj.position.y
        rsp = True 
        if x < self.x0 or x > self.x1:
            rsp = False
        if y < self.y0 or y > self.y1:
            rsp = False
        return rsp
