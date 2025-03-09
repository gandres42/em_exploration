import ss2d # type: ignore
from utils import *
from threading import Thread
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from threading import Lock
from scipy.spatial.transform import Rotation as R
import scipy.spatial.distance as distance
import json

DISCOVERY_DISTANCE = .5

# TODO read landmarks from config file, pre-populate landmark_poses_undiscovered

# handles moving in sim, adjustment of local goal poses based on lidar surroundings
class Simulator2D(Node):
    def __init__(self, landmark_dict):
        # em exploration config
        self.control_model = None
        self.environment = None
        self.sensor_model = None
        self.vehicle = None
        
        # ros position used for converting emmax odometry to nav2 target poses
        self.ros_pose = ss2d.Pose2(0, 0, 0)
        self.ros_map = None
        self.undiscovered_landmarks = landmark_dict
        self.discovered_landmarks = landmark_dict

        # ros subscriber callbacks
        super().__init__('emmax_bridge')
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
        
    def __map_callback__(self, msg):
        self.ros_map = msg


    def __get_map_meters__(self, x, y):
        res = self.ros_map.info.resolution  # meters per cell
        res = self.ros_map.info.resolution
        width = self.ros_map.info.width
        height = self.ros_map.info.height
        origin = self.ros_map.info.origin.position

        # Convert world coordinates to grid indices
        cell_x = int((x - origin.x) / res)
        cell_y = int((y - origin.y) / res)

        # Check if within map bounds
        if 0 <= cell_x < width and 0 <= cell_y < height:
            index = cell_y * width + cell_x
            return self.ros_map.data[index]
        return -1
        

    def __odom_callback__(self, msg):
        # TODO compare radius to discover landmarks here
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        euler_orientation = R.from_quat((orientation.w, orientation.x, orientation.y, orientation.z)).as_euler('xyz', degrees=False)
        self.ros_pose = ss2d.Pose2(position.x, position.y, euler_orientation[0])
        
        # print(self.ros_map)

        if self.ros_map is None:
            return
        
        rig_undiscovered = list(self.undiscovered_landmarks.items())
        for landmark in rig_undiscovered:
            key, pos = landmark
            # print(key, pos)
            # print(self.__get_map_meters__(pos[0], pos[1]))
            if self.__get_map_meters__(pos[0], pos[1]) != -1:
                print(f"found landmark {key}")
                self.discovered_landmarks[key] = pos
                del self.discovered_landmarks[key]

                

    def initialize_vehicle(self, ss2d_pose):
        pass

    # return discovered landmark-key pairs
    def measure(self):
        pass

    # set navigation goal based on current pose + odom
    def move(self, odom, noise):
        pass
    
    def pprint(self):
        pass

    # generate random landmarks
    def random_landmarks(self, landmarks, num_landmarks, environment_params):
        pass

if __name__ == '__main__':
    with open('map_config.json') as json_file:
        landmarks = json.load(json_file)
    rclpy.init()
    node = Simulator2D(landmarks)
    try:
        rclpy.spin(node)  # Keep node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
