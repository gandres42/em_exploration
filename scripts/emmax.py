import sys
from pyplanner2d import *
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from scipy.spatial.transform import Rotation as R
import rclpy
from threading import Thread
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
import time
from scipy.spatial import distance
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.parameter import Parameter
from PIL import Image

from pgm_parser import ROSMapReader

ROS_WIDTH = 2.625
ROS_HEIGHT = 2.85
ROS_RESOLUTION = .05

HEIGHT_SCALE = ROS_HEIGHT / 40
WIDTH_SCALE = ROS_WIDTH / 40


class EMContoller():
    def __init__(self, config_file):
        self.pose = ss2d.Pose2(0, 0, 0)
        self.ros_map = None
        self.config_file = config_file
        plt.ion()
        self.fig, self.ax = plt.subplots(1, 1)

    def explore(self, max_steps, verbose=False, save_history=False):
        config = load_config(self.config_file)
        range_noise = math.radians(0.1)
        config.set('Sensor Model', 'range_noise', str(range_noise))

        explorer = EMExplorer(config, verbose, save_history)

        start_time = time.monotonic()

        for step in range(max_steps):
            if step < 4:
                odom = 0, 0, math.pi / 2.0
                explorer.simulate(odom, core=True)
            else:
                result = explorer.plan()
                if result == planner2d.EMPlanner2D.OptimizationResult.SAMPLING_FAILURE:
                    explorer.simulate((0, 0, math.pi / 4), True)
                elif result == planner2d.EMPlanner2D.OptimizationResult.NO_SOLUTION:
                    break
                elif result == planner2d.EMPlanner2D.OptimizationResult.TERMINATION:
                    break
                else:
                    # make move and get pose
                    explorer.follow_dubins_path(5)
                    pose = explorer._sim.vehicle
                    # move to same pose in nav2
                    self.ax.clear()
                    plot_environment(explorer._sim.environment, label=False, ax=self.ax)
                    plot_pose(explorer._sim.vehicle, explorer._sensor_params, ax=self.ax)
                    plot_map(explorer._slam.map, ax=self.ax)
                    plot_virtual_map(explorer._virtual_map, explorer._map_params, ax=self.ax)
                    plt.draw()
                    plt.pause(0.1)
        
        print(f"Exploration time: {time.monotonic() - start_time}")
        exit()

if __name__ == '__main__':
    config_file = sys.path[0] + '/configs/turtlehouse.ini'
    node = EMContoller(config_file)
    node.explore(100)