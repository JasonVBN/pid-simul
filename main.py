'''
simulate controlling motor throttle to hover drone at 10m
(only care abt altitude, not orientation)
process var: altitude
control var: motor throttle (~ thrust)
'''

import matplotlib.pyplot as plt

TARGET = 10
TIME_STEP = 1 #secs
SIM_TIME = 100 #secs
MASS = 2 #kg
GRAV_ACCEL = 10 #m/s^2


def throttle2thrust(throttle):
    assert 0<=throttle<=100

    #just assume thrust (N) proportional to throttle (%) for now
    #100% throttle = 50 N thrust
    return 0.5*throttle

clamp = lambda x: max(0, min(100,x))
class Controller:
    def __init__(self):
        self.kp = 0.5
    
    def run(self, target, current):
        '''returns throttle %'''
        err = target-current #positive if lower

        return clamp(self.kp*err + 40)

alt_hist=[]
vel_hist=[]
throttle_hist=[]
con=Controller()
t=0
alt=0
vel=0
while t<SIM_TIME:
    throttle = con.run(TARGET,alt)
    thrust = throttle2thrust(throttle)
    net_force = thrust-(MASS*GRAV_ACCEL)
    accel = net_force/MASS

    #double integrate (basically)
    vel += TIME_STEP*accel
    alt += TIME_STEP*vel

    alt_hist.append(alt)
    vel_hist.append(vel)
    throttle_hist.append(throttle)

    t += TIME_STEP

fig, (ax1,ax2,ax3) = plt.subplots(nrows=3, ncols=1, 
    sharex=True,) # layout="constrained"
ax1.plot(alt_hist)
ax1.set_title("altitude")
ax2.plot(vel_hist)
ax2.set_title("velocity")
ax3.plot(throttle_hist)
ax3.set_title("throttle")

plt.tight_layout()
plt.show()