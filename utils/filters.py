# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
class ab_filt():
    def __init__(self,xv=[0,0],alpha=0.5,beta=0.1):
        self.alpha = alpha
        self.beta = beta
        self.reset(xv)

    def reset(self,xv):
        self.x,self.v=xv

    def __call__(self,xm,dt=1.0):
        self.x += self.v*dt

        rk = xm - self.x

        self.x += self.alpha * rk
        self.v += ( self.beta * rk ) / dt
        return (self.x,self.v)



