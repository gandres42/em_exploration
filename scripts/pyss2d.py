import math
from configparser import SafeConfigParser

import numpy as np
from scipy.stats.distributions import chi2

import ss2d
from utils import *

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


class SS2D(object):
    def __init__(self, config, verbose=False):

        # check if config is valid format
        if isinstance(config, str):
            self._config = load_config(config)
        elif isinstance(config, SafeConfigParser):
            self._config = config
        else:
            print('Config type not supported!')

        # load in parameters
        self._sensor_params = read_sensor_params(self._config)
        self._control_params = read_control_params(self._config)
        self._environment_params = read_environment_params(self._config)
        self._map_params = read_map_params(self._config)
        self._virtual_map_params = read_virtual_map_params(self._config, self._map_params)

        # initialize start pose
        x0 = self._config.getfloat('Simulator', 'x0')
        y0 = self._config.getfloat('Simulator', 'y0')
        theta0 = math.radians(self._config.getfloat('Simulator', 'theta0'))

        # initialize start pose uncertainty
        sigma_x0 = self._config.getfloat('Simulator', 'sigma_x0')
        sigma_y0 = self._config.getfloat('Simulator', 'sigma_y0')
        sigma_theta0 = math.radians(self._config.getfloat('Simulator', 'sigma_theta0'))

        # get number of landmarks to be placed
        num_random_landmarks = self._config.getint('Simulator', 'num')

        # if don't want fully random result, use a seed provided in config
        seed = self._config.getint('Simulator', 'seed')
        if seed < 0:
            seed = int(time() * 1e6)

        # TODO replace simulator
        self._sim = ss2d.Simulator2D(self._sensor_params, self._control_params, seed)

        # initialize sim, sla, and virtual map.  leave map, we only care about the simulator
        self._sim.initialize_vehicle(ss2d.Pose2(x0, y0, theta0))
        self._slam = ss2d.SLAM2D(self._map_params)
        self._virtual_map = ss2d.VirtualMap(self._virtual_map_params, seed)

        # add landmarks if their locations are included in the config
        if self._config.has_section('Landmarks') and self._config.has_option('Landmarks', 'x') and \
                self._config.has_option('Landmarks', 'y'):
            x = eval(self._config.get('Landmarks', 'x'))
            y = eval(self._config.get('Landmarks', 'y'))
            landmarks = []
            for xi, yi in zip(x, y):
                landmarks.append(ss2d.Point2(xi, yi))
            self._sim.random_landmarks(landmarks, num_random_landmarks, self._environment_params)
        # otherwise, generate random landmarks
        else:
            self._sim.random_landmarks([], num_random_landmarks, self._environment_params)

        # configure verbose logging
        self.verbose = verbose
        if self.verbose:
            self._sim.pprint()
            self._virtual_map_params.pprint()
            self._slam.pprint()

        # initial state including pose and belief
        initial_state = ss2d.VehicleBeliefState(self._sim.vehicle,
                                                np.diag([1.0 / sigma_x0 ** 2, 1.0 / sigma_y0 ** 2,
                                                         1.0 / sigma_theta0 ** 2]))

        self.step = 0
        self._cleared = True
        self._da_ground_truth = {}

        self._slam.add_prior(initial_state)
        self.measure()
        self.optimize()
        self.step += 1

    # move in simulation and update SLAM with odometry
    def move(self, odom):
        odom = ss2d.Pose2(odom[0], odom[1], odom[2])
        _, self._control_state = self._sim.move(odom, True)
        self._slam.add_odometry(self._control_state)

    # get range and bearing to landmarks
    def measure(self):
        self._measurements = self._sim.measure()

        # if self._measurements != []:
        #     measurement = self._measurements[0][1]
        #     new_pose = ss2d.Pose2(0, 0, 0)

        for key, m in self._measurements:
            self._slam.add_measurement(key, m)
        return

    # optimize slam graph using new measurements
    def optimize(self):
        self._slam.optimize(update_covariance=True)

    # update probabilities
    def update_virtual_map(self, update_probability=False, update_information=True):
        if update_probability:
            self._virtual_map.update_probability(self._slam, self._sim.sensor_model)

        if update_information:
            self._virtual_map.update_information(self._slam.map, self._sim.sensor_model)

    # one simulation step with obstacle avoidance
    def simulate(self, odom, core=True):
        # move by provided odometry
        self.move(odom)

        # avoid obstacles?
        obstacle = False
        ###############################
        measurements = self._sim.measure()
        landmarks = [key for key, landmark in self._slam.map.iter_landmarks()]
        for key, m in measurements:
            if self._cleared:
                if m.range < self._environment_params.safe_distance:
                    obstacle = True
                    self._cleared = False
                    break
            else:
                if key not in landmarks and m.range < self._environment_params.safe_distance:
                    obstacle = True
                    self._cleared = False
                    break
        if not obstacle and core:
            self._cleared = True
        ###############################

        if not core and not obstacle:
            return obstacle

        self.step += 1
        self.measure()
        self.optimize()
        self.update_virtual_map(True, True)
        return obstacle

    # one simulation step without extra avoidance
    def simulate_simple(self, odom):
        self.move(odom)
        self.measure()
        self.optimize()
        self.update_virtual_map(True, True)

        self.step += 1

    @property
    def distance(self):
        return self._slam.map.distance

    @property
    def vehicle_position(self):
        return self._slam.map.get_current_vehicle().pose

    @property
    def map(self):
        return self._slam.map

    @property
    def environment(self):
        return self._sim.environment

if __name__ == '__main__':
    import sys

    ss = SS2D(sys.path[0] + '/configs/pyss2d.ini')
    # ss.savefig()

    for step in range(120):
        if step == 10 or step == 20 or step == 40 or step == 60 or step == 80 or step == 100:
            odom = 0, 0, math.pi / 2.0
        else:
            odom = 1, 0, 0

        ss.simulate_simple(odom)