import sys
from scripts.potentially_irrelevant.pyplanner2d import *
import matplotlib
from pprint import pp as pprint
import time

# explorer = EMExplorer(sys.path[0] + '/isrr2017_structured.ini', )
# pprint(dir(explorer.map))
# trajectory = list(explorer.map.iter_trajectory())
# pprint(dir(trajectory[0].pose))

config = load_config('isrr2017_structured.ini')
sensor_params = read_sensor_params(config)
control_params = read_control_params(config)
map_params = read_map_params(config)
virt_map_params = read_virtual_map_params(config, map_params)
# sim = ss2d.VirtualMap(sensor_params, control_params)
sim = ss2d.VirtualMap(virt_map_params, time.monotonic_ns())

pprint(dir(sim))