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
from nav_landmark import BearingRangeSensorModelMeasurement

OBSERVATION_DISTANCE = 3

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
        self.discovered_landmarks = {}

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
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        euler_orientation = R.from_quat((orientation.w, orientation.x, orientation.y, orientation.z)).as_euler('xyz', degrees=False)
        self.ros_pose = ss2d.Pose2(position.x, position.y, euler_orientation[0])

        if self.ros_map is None:
            return
        
        # discover landmarks when they're first sighted
        rig_undiscovered = list(self.undiscovered_landmarks.items())
        for landmark in rig_undiscovered:
            key, pos = landmark

            if self.__get_map_meters__(pos[0], pos[1]) != -1:
                self.discovered_landmarks[key] = pos
                del self.undiscovered_landmarks[key]
                self.get_logger().info(f"Found landmark {key}")
        
        print(self.measure())

    def __calculate_range_and_bearing__(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2

        dx = x2 - x1
        dy = y2 - y1

        # Calculate range (distance)
        distance = math.sqrt(dx**2 + dy**2)

        # Calculate bearing (angle in radians)
        bearing_rad = math.atan2(dy, dx)

        return distance, bearing_rad


    def initialize_vehicle(self, ss2d_pose):
        pass


    # return discovered landmark-key pairs
    def measure(self):
        robot_pos = (self.ros_pose.x, self.ros_pose.y)

        # TODO line-of-sight detection
        sighted_landmarks = {}
        for landmark in self.discovered_landmarks.items():
            key, pos = landmark
            if distance.euclidean(robot_pos, pos) < OBSERVATION_DISTANCE:
                sighted_landmarks[key] = pos
        self.get_logger().info(f"Sighted landmarks {sighted_landmarks}")

        bearings = []
        for landmark in sighted_landmarks.items():
            key, pos = landmark
            range_m, bearing = self.__calculate_range_and_bearing__(robot_pos, pos)
            bearings.append((int(key), BearingRangeSensorModelMeasurement(bearing, range_m)))

        return np.array(bearings)

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
