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


class EMContoller(Node):
    def __init__(self, config_file, pgm_file, yaml_file):
        super().__init__('emmax_bridge')
        self.pgm_parser = ROSMapReader(pgm_file, yaml_file)

        self.pose = ss2d.Pose2(0, 0, 0)

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.__odom_callback__,
            10
        )
        self.odom_subscription

        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.__map_callback__,
            10
        )
        self.map_subscription
        self.ros_map = None
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.config_file = config_file

        plt.ion()
        self.fig, self.ax = plt.subplots(1, 1)

    def __odom_callback__(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        euler_orientation = R.from_quat((orientation.w, orientation.x, orientation.y, orientation.z)).as_euler('xyz', degrees=False)
        self.pose = ss2d.Pose2(position.x, position.y, euler_orientation[0])

    def __map_callback__(self, msg):
        self.ros_map = msg

    def get_occupancy_value(self, x: float, y: float) -> int:
        """
        Converts ROS coordinates (meters) to grid cell values from an OccupancyGrid message.

        Args:
            self.ros_map (OccupancyGrid): The OccupancyGrid message.
            x (float): X-coordinate in meters.
            y (float): Y-coordinate in meters.

        Returns:
            int: The occupancy value at the specified coordinates (-1 = unknown, 0 = free, 100 = occupied).
        """
        if self.ros_map == None:
            return None
        resolution = self.ros_map.info.resolution
        origin_x = self.ros_map.info.origin.position.x
        origin_y = self.ros_map.info.origin.position.y
        width = self.ros_map.info.width

        # Convert coordinates to grid indices
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)

        # Check if indices are within grid bounds
        if 0 <= grid_x < width and 0 <= grid_y < self.ros_map.info.height:
            # Calculate index in data array (row-major order)
            index = grid_y * width + grid_x
            return self.ros_map.data[index]
        else:
            return -1  # Unknown/invalid value


    def send_goal_and_wait(self, pose):
        """Send a goal and block until navigation is complete."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.client.wait_for_server()
        goal_future = self.client.send_goal_async(goal_msg)

        # Use executor for blocking call
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin_until_future_complete(goal_future)

        goal_handle = goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected (invalid or unreachable point).')
            return False  # Goal rejected immediately

        self.get_logger().info(f'Goal accepted, navigating to {pose.pose.position.x, pose.pose.position.y}')

        # Wait for result
        result_future = goal_handle.get_result_async()
        executor.spin_until_future_complete(result_future)

        # Check result status
        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded!')
            return True
        elif result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Navigation failed: Unreachable point.')
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation canceled.')
        else:
            self.get_logger().warn(f'Unknown status code: {result.status}')

        return False

    def move(self, odom: ss2d.Pose2):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"  # Adjust as needed

        # Set position
        msg.pose.position.x = odom.y
        msg.pose.position.y = odom.x
        msg.pose.position.z = 0.0  # Assuming flat ground

        # Convert theta (rotation about x-axis) to quaternion
        qx = math.sin(odom.theta / 2.0)
        qw = math.cos(odom.theta / 2.0)

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        
        while self.ros_map is None:
            time.sleep(0.01)
        
        self.send_goal_and_wait(msg)

    def is_safe(self, x, y, min_distance_cells):
        width = self.ros_map.info.width
        height = self.ros_map.info.height

        for dx in range(-min_distance_cells, min_distance_cells + 1):
            for dy in range(-min_distance_cells, min_distance_cells + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    index = ny * width + nx
                    if self.ros_map.data[index] == 254:  # Occupied cell
                        return False
        return True

    def find_nearest_unoccupied(self, x: float, y: float):
        resolution = self.ros_map.info.resolution
        origin_x = self.ros_map.info.origin.position.x
        origin_y = self.ros_map.info.origin.position.y
        width = self.ros_map.info.width
        height = self.ros_map.info.height

        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)

        min_distance = float('inf')
        nearest_point = (x, y)

        min_distance_cells = int(0.25 / resolution)

        for i in range(height):
            for j in range(width):
                index = i * width + j
                if self.ros_map.data[index] == 0 and self.is_safe(j, i, min_distance_cells):
                    dist = math.sqrt((grid_x - j) ** 2 + (grid_y - i) ** 2)
                    if dist < min_distance:
                        min_distance = dist
                        nearest_point = (j * resolution + origin_x, i * resolution + origin_y)

        return nearest_point

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

                    ros_x = pose.x / 2.75
                    ros_y = pose.y / 2.75
                    occupancy = None
                    while occupancy == None:
                        occupancy = self.get_occupancy_value(ros_x, ros_y)
                        time.sleep(0.1)
                    print(occupancy)
                    ros_x, ros_y = self.find_nearest_unoccupied(ros_x, ros_y)
                    print(ros_x, ros_y)

                    # print(ros_x, ros_y)
                    # move to same pose in nav2
                    self.ax.clear()
                    plot_environment(explorer._sim.environment, label=False, ax=self.ax)
                    plot_pose(explorer._sim.vehicle, explorer._sensor_params, ax=self.ax)
                    plot_map(explorer._slam.map, ax=self.ax)
                    plot_virtual_map(explorer._virtual_map, explorer._map_params, ax=self.ax)
                    plt.draw()
                    plt.pause(0.1)
                    input()
                    # self.move(ss2d.Pose2(ros_x, ros_y, 0))
        
        print(f"Exploration time: {time.monotonic() - start_time}")
        exit()

if __name__ == '__main__':
    config_file = sys.path[0] + '/configs/turtlehouse.ini'
    pgm_file = sys.path[0] + '/maps/house/turtlehouse.pgm'
    yaml_file = sys.path[0] + '/maps/house/turtlehouse.yaml'
    rclpy.init()
    
    # create explorer object
    node = EMContoller(config_file, pgm_file, yaml_file)
    # print(em_to_ros(ss2d.Pose2(0, 0, 0)))

    # create ros update thread
    spin_thread = Thread(target=rclpy.spin, args=(node, ), daemon=True)
    spin_thread.start()

    # start main exploration loop
    node.explore(100)

    # /home/gavin/Git/emmax_ws/scripts/maps/house/turtlehouse.pgm