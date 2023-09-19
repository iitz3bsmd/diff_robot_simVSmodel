import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Vector3
import matplotlib.pyplot as plt


class sim_controller(Node):
    def __init__(self):
        super().__init__('sim_controller')

        #path
        self.commands = [(1.0,1.0), (1.5,1.0), (1.0,1.5), (1.0, 1.0)]

        self.l_w_pub = self.create_publisher(Float32, 'l_w', 10)
        self.r_w_pub = self.create_publisher(Float32, 'r_w', 10)
        self.start_sim_pub = self.create_publisher(Bool, '/startSimulation', 10)
        self.stop_sim_pub = self.create_publisher(Bool, '/stopSimulation', 10)
        self.enable_step_pub = self.create_publisher(Bool, '/enableSyncMode', 10)
        self.next_step_pub = self.create_publisher(Bool, '/triggerNextStep', 10)

        self.timer = self.create_timer(0.05, self.compare)

        self.pos_sub = self.create_subscription(Vector3, '/pos', self.pos_callback, 10)
        self.step_done_sub = self.create_subscription(Bool, '/simulationStepDone',self.step_done_callback, 10)
        self.sim_time_sub = self.create_subscription(Float32, '/simulationTime', self.sim_time_callback, 10)

        self.l_w = Float32()
        self.r_w = Float32()
        self.r = 0.05
        self.d = 0.15
        self.sim_x = 0
        self.sim_y = 0
        self.model_x = 0
        self.model_y = 0
        self.euler_theta = 0
        self.dt = 0.05
        self.t = 0
        self.step_done = False
        self.euler_res = []
        self.rk_res = []
        self.sim_res = []
        self.i = 0
        self.command_i = 0
        
        self.true_msg = Bool()
        self.true_msg.data = True
        self.enable_step_pub.publish(self.true_msg)
        self.start_sim_pub.publish(self.true_msg)

        self.next_step()

    def pos_callback(self, msg):
        self.sim_x = msg.x
        self.sim_y = msg.y

    def step_done_callback(self, msg):
        self.step_done = msg.data

    def sim_time_callback(self, msg):
        self.t = msg.data

    def next_step(self):
        self.next_step_pub.publish(self.true_msg)

    def omega_pub(self):
        self.l_w_pub.publish(self.l_w)
        self.r_w_pub.publish(self.r_w)

    def compare(self):
        if self.step_done == False:
            return
        
        self.sim_res.append((self.sim_x,self.sim_y))

        self.step_done = False

        if self.i > 200:
            self.command_i += 1
            self.i = 0

        if self.command_i > len(self.commands)-1:
            self.stop_sim_pub.publish(self.true_msg)
            x_euler = [x[0] for x in self.euler_res]
            y_euler = [y[1] for y in self.euler_res]
            x_sim = [x[0] for x in self.sim_res]
            y_sim = [y[1] for y in self.sim_res]
            plt.plot(x_euler, y_euler, color='r', label='Model')
            plt.plot(x_sim, y_sim, color='b', label='Simulation')
            plt.legend()
            plt.show()
            self.destroy_node()

        omega_r, omega_l = self.commands[self.command_i]

        #euler method
        x_dot = 0.5 * self.r * (omega_r + omega_l) * math.cos(self.euler_theta)
        y_dot = 0.5 * self.r * (omega_r + omega_l) * math.sin(self.euler_theta)

        theta_dot = self.r * (omega_r - omega_l) / self.d
        self.euler_theta = self.euler_theta + theta_dot * self.dt

        self.model_x = self.model_x + x_dot * self.dt
        self.model_y = self.model_y + y_dot * self.dt

        self.euler_res.append((self.model_x,self.model_y))

        if self.i == 0:
            self.l_w.data = -omega_l
            self.r_w.data = -omega_r
            self.omega_pub()
        self.next_step()
        self.i += 1
        

def main(args=None):

    rclpy.init(args=args)
    sim = sim_controller()
    rclpy.spin(sim)
    sim.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





