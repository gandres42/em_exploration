import ss2d # type: ignore
from utils import *
import numpy as np

# class representing range and bearing to landmarks, as well as their unique ID

class BearingRangeSensorModelMeasurement:
    def __init__(self, bearing, range, sigmas=np.array([0.00349066, 0.1])):
        # uncertainties
        self.sigmas = sigmas
        # radians bearing to landmark
        self.bearing = bearing

        # leave as false for now to avoid implementation
        self.has_jacobian = False

        # range to landmark (m?)
        self.range = range

    def pprint(self):
        pass

    def transform_from(self, pose: ss2d.Pose2):
        return ss2d.Pose2()