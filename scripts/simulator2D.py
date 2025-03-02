# reimplementation of ss2d.Simulator2D with Nav2 support

class Simulator2D:
    def __init__(self):
        self.control_model = None
        self.environment = None
        self.sensor_model = None
        self.vehicle = None

    def initialize_vehicle(self, ss2d_pose):
        pass

    def measure(self):
        pass

    def move(self, odom, noise):
        pass

    def pprint(self):
        pass

    def random_landmarks(self, landmarks, num_landmarks, environment_params):
        pass