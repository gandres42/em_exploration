import sys
from pyplanner2d import *
import matplotlib.lines as mlines
import rclpy
import tempfile

def explore_isrr2017_structured(config_file, max_steps, verbose=False, save_history=False, save_fig=True):
    config = load_config(config_file)
    range_noise = math.radians(0.1)
    config.set('Sensor Model', 'range_noise', str(range_noise))

    explorer = EMExplorer(config, verbose, save_history)

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
                explorer.follow_dubins_path(5)

# config_file = sys.path[0] + '/isrr2017_random.ini'
# explore_isrr2017_random(config_file, 100, True, False, True)
rclpy.init()
config_file = sys.path[0] + '/configs/isrr2017_structured.ini'
explore_isrr2017_structured(config_file, 100, True, False, True)

