import numpy as np
import matplotlib.pyplot as plt
import random
###### ASK ABOUT 1st Order Derivative Filter and (integral) anti windup

class Mass_Spring_Dampner:

    def __init__(self, mass, spring_constant, damping_constant, noise_std=0.02):
        # test to see effects are altering noise 
        self.tstart = 0.0
        self.tstop = 600.0
        self.Ts = 0.01
        # set up initial time and time step
        self.mass = mass
        self.spring_constant = spring_constant
        self.damping_constant = damping_constant
        self.noise_std = noise_std # standard deviation of measurement noise (I assume this is what is throwing off my other code)
        self.N = int((self.tstop-self.tstart)/self.Ts) # Simulation Length
        self.x1 = np.zeros(self.N+2) # Initialize array with called x1 with N+2 elements, we use np.zeroes since mass starts are 0
        self.x2 = np.zeros(self.N+2) # Initialize array with called x1 with N+2 elements, we use np.zeroes since velocity starts are 0
        self.scalar_force = 0.0 

    def set_initial_value(self, initial_position, initial_velocity):
        self.x1[0] = initial_position
        self.x2[0] = initial_velocity

    def set_force(self,scalar_force):
        self.scalar_force = scalar_force 

    def _state_equation(self, _, k):
        position = self.x1[k]
        velocity = self.x2[k]
        spring_acceleration = (self.spring_constant / self.mass) * position
        damper_acceleration = (self.damping_constant / self.mass) * velocity
        external_acceleration = (1.0 / self.mass) * self.scalar_force
        acceleration = (- spring_acceleration
                        - damper_acceleration
                        + external_acceleration)
        dxdt = [velocity, acceleration]
        return dxdt
    
    def get_measurement(self,k):
        self.tstart = self.tstart + self.Ts
        # Solve Ordinary Differential Equation for current states
        dxdt = self._state_equation(0,k)
        self.x1[k+1] = self.x1[k] + dxdt[0] * self.Ts
        self.x2[k+1] = self.x2[k] + dxdt[1] * self.Ts
        # this essentially makes it so that the next position/velocity is updated 
        # velocity * time step will tell us our change in position, and acceleration * time step will tell us our change in velocity

        position = self.x1[k+1]
        velocity = self.x2[k+1]
        noise = random.gauss(0,self.noise_std)
        measured_position = position + noise 
        return self.tstart,measured_position,velocity
    
class PID:

    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp # float proportional gain
        self.Ki = Ki # float integral gain
        self.Kd = Kd # float derivative gain
        self.t0 = None # float/None Initial Time. None will reset time
        self.e0 = None # float/None Initial Error. None will reset error
        self.i0 = None # float/None Initial Integral. None will reset integral

    def _if_None_Values(self,t,e):
        if self.t0 is None:
            self.t0 = t 
        if self.e0 is None:
            self.e0 = e 
        if self.i0 is None:
            self.i0 = 0.0 
    
    def integrate(self,t,e):
        ## t reprents current time and e represents error signal 

        self._if_None_Values(t,e)

        # Calulate time step
        dt = t - self.t0 
        # Calculate proportional term
        p = self.Kp * e 
        # Calculate integral term
        i = self.i0 + dt * self.Ki * e 
        # Calculate derivative term 
        d = 0.0 
        # set initial value for next cycle 
        self.t0 = t 
        self.e0 = e 
        self.i0 = i 
        return p + i + d

my_system = Mass_Spring_Dampner(mass = 20.0, spring_constant=2,damping_constant=4)
my_system.set_initial_value(initial_position=0,initial_velocity=0)

my_pid = PID(Kp=20.0,Ki=0.1,Kd=10.0)

time, position_lst, velocity_lst,force_lst = [],[],[],[]

# my target
how_long = my_system.N/4
print(how_long)
list_of_targets = [10,5,20,2]
target = list_of_targets[0]
next = 1
count = 0

for k in range(my_system.N + 1):
    tstart,position,velocity = my_system.get_measurement(k)

    if count == how_long and target != list_of_targets[-1]:
        print(position_lst[k-1])
        target = list_of_targets[next]
        next += 1
        count = 0

    control_input = my_pid.integrate(tstart,target-position)

    my_system.set_force(control_input)
    time.append(tstart)
    position_lst.append(position)
    velocity_lst.append(velocity)
    force_lst.append(control_input)
    count += 1

print(position_lst[-1])
plt.plot(time,position_lst)
# plt.plot(time,velocity_lst)
plt.title("Simulation of Mass-Spring-Damper System")
plt.xlabel('t[s]')
plt.ylabel('x(t)')
plt.grid()
plt.legend(['x1','x2'])
plt.show()



