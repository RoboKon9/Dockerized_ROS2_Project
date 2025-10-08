import rclpy
from rclpy.node import Node
from sensor_msgs.msg._joint_state import JointState
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import time

class JointTraj(Node):
    def __init__(self):
        super().__init__('joint_traj_subscriber')
        self.subscription = self.create_subscription(
            JointState,                  
            'joint_traj',               
            self.joint_listener_callback,  
            10                       
        )
        self.subscription 
        self.joint_angle_2 = []  #chose to plot the trajectory of the second joint.
        self.time = []
        self.initial_time = time.time()

        # Plot initialization
        plt.ion() #interactive mode
        self.fig, self.ax = plt.subplots() #initialize figure
        self.line: Line2D
        self.line, = self.ax.plot(self.time,self.joint_angle_2)
        



    def joint_listener_callback(self, msg):
        #plotting joint 2
        joint_angles = msg.position 
        self.joint_angle_2.append(joint_angles[1])
        self.time.append(time.time()-self.initial_time) 
        self.line.set_xdata(self.time)
        self.line.set_ydata(self.joint_angle_2)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()




        

def main(args=None):
    rclpy.init(args=args)
    node = JointTraj()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()