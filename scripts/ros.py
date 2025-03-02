import sys
from pyplanner2d import *
import matplotlib
from pprint import pp as pprint

# explorer = EMExplorer(sys.path[0] + '/isrr2017_structured.ini', )
# pprint(dir(explorer.map))
# trajectory = list(explorer.map.iter_trajectory())
# pprint(dir(trajectory[0].pose))

config = load_config('isrr2017_structured.ini')
sensor_params = read_sensor_params(config)
control_params = read_control_params(config)
sim = ss2d.Simulator2D(sensor_params, control_params)

pprint(dir(sim))