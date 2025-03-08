from threading import Thread
import rclpy
import struct
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Header
import ss2d # type: ignore
from utils import *


# class Simulator2D:
#     def __init__(self):
#         self.control_model = None
#         self.environment = None
#         self.sensor_model = None
#         self.vehicle = None

#     def initialize_vehicle(self, ss2d_pose):
#         pass

#     def measure(self):
#         pass

#     def move(self, odom, noise):
#         pass

#     def pprint(self):
#         pass

#     def random_landmarks(self, landmarks, num_landmarks, environment_params):
#         pass


def read_sensor_params(config):
    sensor_params = ss2d.BearingRangeSensorModelParameter()
    sensor_params.bearing_noise = math.radians(config.getfloat('Sensor Model', 'bearing_noise'))
    # sensor_params.range_noise = math.radians(config.getfloat('Sensor Model', 'range_noise'))
    sensor_params.range_noise = config.getfloat('Sensor Model', 'range_noise')
    sensor_params.min_bearing = math.radians(config.getfloat('Sensor Model', 'min_bearing'))
    sensor_params.max_bearing = math.radians(config.getfloat('Sensor Model', 'max_bearing'))
    sensor_params.min_range = config.getfloat('Sensor Model', 'min_range')
    sensor_params.max_range = config.getfloat('Sensor Model', 'max_range')
    return sensor_params

def read_control_params(config):
    control_params = ss2d.SimpleControlModelParameter()
    control_params.rotation_noise = math.radians(config.getfloat('Control Model', 'rotation_noise'))
    control_params.translation_noise = config.getfloat('Control Model', 'translation_noise')
    return control_params

def read_environment_params(config):
    environment_params = ss2d.EnvironmentParameter()
    environment_params.min_x = config.getfloat('Environment', 'min_x')
    environment_params.max_x = config.getfloat('Environment', 'max_x')
    environment_params.min_y = config.getfloat('Environment', 'min_y')
    environment_params.max_y = config.getfloat('Environment', 'max_y')
    environment_params.safe_distance = config.getfloat('Environment', 'safe_distance')
    return environment_params

def read_virtual_map_params(config, map_params):
    virtual_map_params = ss2d.VirtualMapParameter(map_params)
    virtual_map_params.resolution = config.getfloat('Virtual Map', 'resolution')
    virtual_map_params.sigma0 = config.getfloat('Virtual Map', 'sigma0')
    virtual_map_params.num_samples = config.getint('Virtual Map', 'num_samples')
    return virtual_map_params

def read_map_params(config, ext=5.0):
    map_params = ss2d.EnvironmentParameter()
    map_params.min_x = config.getfloat('Environment', 'min_x') - ext
    map_params.max_x = config.getfloat('Environment', 'max_x') + ext
    map_params.min_y = config.getfloat('Environment', 'min_y') - ext
    map_params.max_y = config.getfloat('Environment', 'max_y') + ext
    map_params.safe_distance = config.getfloat('Environment', 'safe_distance')
    return map_params


# plan: hybrid stack
#   em provides movement odometry to nav2, which is using SLAM at the same time to avoid obstacles and handle local movements
#   when em "sees" an obstacle, it adds it to it's slam
#   we ignore SLAM being done by nav add only give position changes as current_estimated_position + em slam odometry

class NavSimulator2D(Node):
    def __init__(self, sensor_params, control_params):
        super().__init__('landmark_publisher')
        self._landmark_publisher = self.create_publisher(PointCloud2, 'pointcloud2_topic', 10)
        self._pose_publisher = self.create_publisher(Pose2D, '')

        self._landmarks = [(0, 0)]
        self._discovered_landmarks = {}
        self._sensor_params = sensor_params
        
        # self.
        
        # self.environment = ss2d.Environment(environment_params)

    def _landmark_thread():
        pass

    # return (landmark_key, BearingRangeSensorModelMeasurement) for each landmark discovered
    def measure():
        pass

if __name__ == '__main__':
    NavSimulator2D('.configs/isrr2017_structured.ini')