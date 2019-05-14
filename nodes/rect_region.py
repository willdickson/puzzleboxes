
class RectRegion(object):

    def __init__(self, param):
        self.param = dict(param)

    def contains(self, obj):
        x,y = obj.position.x, obj.position.y
        rsp = True 
        if x < self.param['x0'] or x > self.param['x1']:
            rsp = False
        if y < self.param['y0'] or y > self.param['y1']:
            rsp = False
        return rsp
