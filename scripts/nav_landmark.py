import ss2d # type: ignore
from utils import *
import numpy as np
import math

# represents range and bearing to landmarks, as well as their unique ID
class BearingRangeSensorModelMeasurement:
    def __init__(self, bearing, range_m, sigmas=np.array([0.00349066, 0.1])):
        # uncertainties
        self.sigmas = sigmas
        # radians bearing to landmark
        self.bearing = bearing

        # leave as false for now to avoid implementation
        self.has_jacobian = False

        # range to landmark (m?)
        self.range = range_m

    def pprint(self):
        pass

    def transform_from(self, pose: ss2d.Pose2):
        yaw = pose.theta
        global_angle = yaw + self.bearing
        dx = self.range * math.cos(global_angle)
        dy = self.range * math.sin(global_angle)

        return np.array([dx, dy])


if __name__ == '__main__':
    # ss2d.BearingRangeSensorModelMeasurement()
    print(dir(ss2d.Pose2))
    