import numpy as np
import matplotlib.pyplot as plt
import math
from matplotlib.animation import FuncAnimation

# to do list, add bounds to turning, make deceleration function

class AircraftSimulation:
    def __init__(self,waypoints):
        # self.Iz = 15000  # aircraft moment of inertia (kgm^2) 
        # self.m = 2182  # aircraft taxi weight (kg)
        # self.f = 50
        self.Iz = 10  # aircraft moment of inertia (kgm^2) 
        # moment of inertia is mass * 
        self.m = 1  # aircraft taxi weight (kg)
        # self.f = 5
        # self.list_of_targets = [[0,0],[1,1],[2,3],[3,6],[4,10],[5,15],[6,21],[7,28],[8,36],[9,45],[10,55]]
        self.list_of_targets = waypoints
        self.target_position = self.list_of_targets[1]
        self.x_dots = []
        self.y_dots = []
        for x in waypoints:
            self.x_dots.append(x[0])
            self.y_dots.append(x[1])
        self.next = 1
        # self.Kp = np.array([[1,1,1,1]])
        self.Kp_angle = 50
        #used to be 50
        self.Kp_position = .001
        self.max_turn_rate = np.pi
        self.min_turn_rate = -np.pi
        self.max_f = 10
        self.min_f = -10
        #used to be 50
        self.stop = False

        self.state_vector = np.array([[0], # X
                                    [0],   # Y
                                    [0],   # Vx
                                    [0],   # Vy
                                    [0],   # Psi
                                    [0]])  # Delta Psi
        
        # self.control_vector = np.array([[0],
        #                                 [0],
        #                                 [0],
        #                                 [1/self.Iz]])
        
        # Simulation Parameters
        self.Ts = 0.01
        self.Tstart = 0
        self.Tstop = 150000
        self.N = int((self.Tstop - self.Tstart) / self.Ts)  # Simulation Length  

        self.x_pos = []
        self.y_pos = []
        self.Vx = []
        self.Vy = []
        self.psi = []
        self.psi_rate = []


    
    def step_lat(self,vector,u,f):
        cos = np.cos(vector[4][0])
        if abs(cos) < 1e-10:
            cos = 0
        sin = np.sin(vector[4][0])
        if abs(sin) < 1e-10:
            sin = 0
        step = np.array([[vector[2][0]],
                         [vector[3][0]],
                         [(f/self.m) * cos],
                         [(f/self.m) * sin],
                         [vector[5][0]],
                         [u/self.Iz],
                         ])
        return vector + self.Ts * step
    
    def compute_angle(self,x,y):
        if y == 0:
            if x > 0:
                return 0
            else:
                return np.pi 
        elif x == 0:
            if y > 0:
                return np.pi/2 
            elif y < 0:
                return -np.pi/2 
            
        angle = math.atan2(y, x)

        if x > 0:
            return angle
        elif x < 0:
            if y > 0:
                return angle
            if y < 0: 
                return angle
                
        
        return angle

    def find_waypoint_line(self,x,y):
        # y = mx + b
        if self.next < len(self.list_of_targets):
            end_x = self.list_of_targets[self.next][0]
            end_y = self.list_of_targets[self.next][1]
            # print(end_x,end_y,"end")
            delta_x = end_x -  self.list_of_targets[self.next-1][0]
            delta_y = end_y -  self.list_of_targets[self.next-1][1]
            if delta_y == 0:
                if x > end_x:
                    return True 
            elif delta_x == 0:
                if y > end_y:
                    return True
            else:
                slope = -delta_x/delta_y 
            # print(slope,"slope")
            b = end_y - (slope*end_x)
            # print(b,"b")
            # print((x,y),"x,y")
            if delta_y < 0:
                if y <= ((slope*x) + b):
                # print("True")
                    return True
                else:
                # print("False")
                    return False
            else:
                if y >= ((slope*x) + b):
                    # print("True")
                    return True
                else:
                    # print("False")
                    return False
    
    def run_simulation(self):
        for k in range(self.N + 1):
            if self.stop == True:
                print("last waypoint")
                break
            x_distance = self.target_position[0] - self.state_vector[0][0]
            y_distance = self.target_position[1] - self.state_vector[1][0]

            target_angle = self.compute_angle(x_distance,y_distance)
            if k ==0:
                print(target_angle)
                # print(target_angle)
            angle_error =  (target_angle - self.state_vector[4][0]) - self.state_vector[5][0]

            if y_distance == 0 :
                distance = x_distance
                # print("zero_y")
                distance = distance - self.state_vector[2][0]
            elif x_distance == 0 :
                # print("zero_x")
                distance = y_distance
                distance = distance - self.state_vector[3][0]
            elif self.find_waypoint_line(self.state_vector[0][0],self.state_vector[1][0]):
                # print("passed",self.state_vector[0][0],self.state_vector[1][0])
                self.state_vector = np.array([[self.state_vector[0][0]],
                                              [self.state_vector[1][0]],
                                              [0],
                                              [0],
                                              [self.state_vector[4][0]],
                                              [self.state_vector[5][0]]])
                if self.next < len(self.list_of_targets)-1:
                    self.next += 1
                    self.target_position = self.list_of_targets[self.next]
                    # print(self.target_position)
                elif self.next < len(self.list_of_targets):
                    self.target_position = self.list_of_targets[self.next]
                    self.stop = True

            else:
                distance = math.sqrt(x_distance**2 + y_distance**2)

                current_vel = (math.sqrt(self.state_vector[2][0]**2 + self.state_vector[3][0]**2))
                if self.state_vector[2][0] < 0 or self.state_vector[3][0] < 0:
                    current_vel = - current_vel
                distance = distance - current_vel


                # v_desired = distance / self.Ts

                # velocity_error = v_desired - (math.sqrt(self.state_vector[2][0]**2 + self.state_vector[3][0]**2))

            u = self.Kp_angle * angle_error
            u = max(self.min_turn_rate, min(self.max_turn_rate, u))
            f = distance * self.Kp_position
            f = max(self.min_f, min(self.max_f, f))
            
            self.state_vector  = self.step_lat(self.state_vector,u,f) 

            self.x_pos.append(self.state_vector[0][0])
            self.y_pos.append(self.state_vector[1][0])
            self.Vx.append((self.state_vector[2][0]))
            self.Vy.append((self.state_vector[3][0]))
            self.psi.append(self.state_vector[4][0])
            self.psi_rate.append(self.state_vector[5][0])
        print("ended")
            

    def plot_results_normal(self):
        t = np.arange(self.Tstart, self.Tstop + 1 * self.Ts, self.Ts)
        t = t[:len(self.x_dots)]
        # plt.xlim((10890, 10970))
        # plt.plot(self.x_pos,self.y_pos)
        plt.plot(t,self.x_dots)
        # plt.plot(t,self.y_pos)
        # plt.plot(t, self.psi)
        # plt.plot(t, self.psi_rate)
        # plt.plot(t, self.Vx)
        # plt.plot(t,self.Vy)
        print(self.list_of_targets[-1],"last_one")
        # print(self.state_vector,"vec")
        # plt.plot(t,self.velocity)
        plt.title("Snapshot of X-position")
        # plt.legend(["X","Y"])
        # plt.legend(["Psi", "Psi Rate"])
        # plt.legend(["Vx", "Vy"])
        plt.grid()
        plt.show()

    def plot_results_animated(self):
        t = np.arange(self.Tstart, self.Tstop + 1 * self.Ts, self.Ts)

        fig = plt.figure()
        ax = plt.subplot(1, 1, 1)

        data_skip = 150000
        #150000


        def init_func():
            ax.clear()
            plt.title("Simulation of Simple Fixed-Winged Aircraft System with P Control")
            plt.xlabel('x')
            plt.ylabel('y')
            plt.xlim((-.5, 3.25))
            plt.ylim((-.5, 3.25))
            plt.grid()


        def update_plot(i):
            ax.plot([0,3,0,0],[0,1,1,3],"bo")
            ax.plot(self.x_pos[i:i+data_skip], self.y_pos[i:i+data_skip], color='r')

            # ax.scatter(self.x_pos[i], self.y_pos[i], marker='o', color='r')

            print(i)



        anim = FuncAnimation(fig,
                            update_plot,
                            frames=np.arange(0, len(self.x_pos), data_skip),
                            init_func=init_func,
                            interval=20)

        anim.save('changed_force_to_10.mp4', dpi=150, fps = 30, writer='ffmpeg')

# simulation = AircraftSimulation([[0,0],[1,-1],[2,-2],[3,-3]])
# simulation.run_simulation()
# simulation.plot_results_animated()

