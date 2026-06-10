'''
simulate controlling motor throttle to hover drone at 10m
(only care abt altitude, not orientation)
process var: altitude
control var: motor throttle (~ thrust)
'''

import matplotlib.pyplot as plt
import numpy as np

TARGET = 10
TIME_STEP = .1 #secs
SIM_TIME = 100 #secs
MASS = 2 #kg
GRAV_ACCEL = 10 #m/s^2


def throttle2thrust(throttle):
    assert 0<=throttle<=100

    #just assume thrust (N) proportional to throttle (%) for now
    #100% throttle = 50 N thrust
    return 0.5*throttle

clamp = lambda x,mn,mx: max(mn, min(mx,x))
class Controller:
    def __init__(self, kp=2, ki=0, kd=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.cumul_err = 0
        self.last_err = None
    
    def run(self, target, current):
        '''returns throttle %'''
        err = target-current #positive if lower
        self.cumul_err += TIME_STEP*err
        self.cumul_err = clamp(self.cumul_err, -100,100)

        d_err = 0
        if self.last_err is not None:
            d_err = (err-self.last_err)/TIME_STEP
        self.last_err = err

        return clamp(self.kp*err 
            + self.ki*self.cumul_err
            + self.kd*d_err, 
            0, 100)

t_vals=[]
alt_hist=[]
vel_hist=[]
throttle_hist=[]
cumulerr_hist=[]
def simulate(controller):
    alt_hist.clear()
    vel_hist.clear()
    throttle_hist.clear()
    cumulerr_hist.clear()

    con = Controller(
        controller.kp, controller.ki, controller.kd)
    t = 0
    alt = 0
    vel = 0
    while t < SIM_TIME:
        throttle = con.run(TARGET, alt)
        thrust = throttle2thrust(throttle)
        net_force = thrust - (MASS*GRAV_ACCEL)
        accel = net_force / MASS

        vel += TIME_STEP*accel
        alt += TIME_STEP*vel

        t_vals.append(t)
        alt_hist.append(alt)
        vel_hist.append(vel)
        throttle_hist.append(throttle)
        cumulerr_hist.append(con.cumul_err)

        t += TIME_STEP


def draw(controller):
    simulate(controller)

    plt.close('all')
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(nrows=4, ncols=1, 
                                        sharex=True,
                                        figsize=(9,9))
    
    l1, = ax1.plot(t_vals, alt_hist)
    ax1.set_title('altitude')
    l2, = ax2.plot(t_vals, vel_hist)
    ax2.set_title('velocity')
    l3, = ax3.plot(t_vals, throttle_hist)
    ax3.set_title('throttle')
    l4, = ax4.plot(t_vals, cumulerr_hist)
    ax4.set_title('cumul_err')

    ax1.plot(t_vals, [TARGET]*len(t_vals)) #target line

    plt.subplots_adjust(left=0.1, right=0.9, bottom=0.2, top=0.95)

    # sliders
    from matplotlib.widgets import Slider

    ax_kp = plt.axes([0.1, 0.12, 0.7, 0.03]) #left,bottom,width,height
    ax_ki = plt.axes([0.1, 0.08, 0.7, 0.03])
    ax_kd = plt.axes([0.1, 0.04, 0.7, 0.03])

    s_kp = Slider(ax_kp, 'kp', 0.0, 10.0, valinit=controller.kp)
    s_ki = Slider(ax_ki, 'ki', 0, 1.0, valinit=controller.ki)
    s_kd = Slider(ax_kd, 'kd', 0.0, 5.0, valinit=controller.kd)

    def _update(val):
        controller.kp = s_kp.val
        controller.ki = s_ki.val
        controller.kd = s_kd.val
        simulate(controller)
        l1.set_ydata(alt_hist)
        l2.set_ydata(vel_hist)
        l3.set_ydata(throttle_hist)
        l4.set_ydata(cumulerr_hist)
        # adjust xlimits and ylimits to new data
        n = len(alt_hist)
        for ax, data in ((ax1, alt_hist), 
                        (ax2, vel_hist), 
                        (ax3, throttle_hist)
                        ):
            ymin = min(data)-2
            ymax = max(data)+2
            ax.set_ylim(ymin, ymax)
        fig.canvas.draw_idle()

    s_kp.on_changed(_update)
    s_ki.on_changed(_update)
    s_kd.on_changed(_update)

    plt.show()


if __name__ == '__main__':
    controller = Controller()
    draw(controller)