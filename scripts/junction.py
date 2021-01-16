import os
import enum
import numpy as np
import math
import matplotlib.pyplot as plt
from IPython import display
from typing import List, Set, Dict, Optional
from collections import namedtuple

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.scenario.lanelet import Lanelet
from commonroad.scenario.intersection import Intersection
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet, LaneletType
from commonroad.scenario.traffic_sign import TrafficLight, TrafficLightState

from commonroad_ccosy.geometry.util import chaikins_corner_cutting, resample_polyline
from pycrccosy import CurvilinearCoordinateSystem

draw_params = \
    {"intersection": {
        "draw_intersections": True,
        "draw_incoming_lanelets": True,
        "incoming_lanelets_color": "#3ecbcf",
        "draw_crossings": True,
        "crossings_color": "#b62a55",
        "draw_successors": True,
        "successors_left_color": "#ff00ff",
        "successors_straight_color": "blue",
        "successors_right_color": "#ccff00",
        "show_label": True
    },
    "lanelet": {
        "left_bound_color": "#555555",
        "right_bound_color": "#555555",
        "center_bound_color": "#dddddd",
        "unique_colors": False,
        "draw_stop_line": True,
        "stop_line_color": "#ffffff",
        "draw_line_markings": True,
        "draw_left_bound": True,
        "draw_right_bound": True,
        "draw_center_bound": True,
        "draw_border_vertices": False,
        "draw_start_and_direction": True,
        "colormap_tangent": False,
        "show_label": True,
        "draw_linewidth": 0.5,
        "fill_lanelet": True,
        "facecolor": "#c7c7c7"
    }
}

def extract_intersection(lanelet_network: LaneletNetwork):
    lanelet_ids = set()
    for la1 in lanelet_network.lanelets:
        for la2 in lanelet_network.lanelets:
            if la1.lanelet_id == la2.lanelet_id:
                continue
            lanelets_not_allowed = [la1.adj_left, la1.adj_right] \
                                   + [lanelet_network.find_lanelet_by_id(la_tmp).adj_left for la_tmp in la1.predecessor] \
                                   + [lanelet_network.find_lanelet_by_id(la_tmp).adj_right for la_tmp in la1.predecessor] \
                                + [lanelet_network.find_lanelet_by_id(la_tmp).adj_left for la_tmp in la1.successor] \
                                +[lanelet_network.find_lanelet_by_id(la_tmp).adj_right for la_tmp in la1.successor]
            if la1.adj_left is not None:
                lanelets_not_allowed += lanelet_network.find_lanelet_by_id(la1.adj_left).predecessor
            if la1.adj_left is not None:
                lanelets_not_allowed += lanelet_network.find_lanelet_by_id(la1.adj_left).successor
            if la1.adj_right is not None:
                lanelets_not_allowed += lanelet_network.find_lanelet_by_id(la1.adj_right).predecessor
            if la1.adj_right is not None:
                lanelets_not_allowed += lanelet_network.find_lanelet_by_id(la1.adj_right).successor

            if la1.convert_to_polygon().shapely_object.intersection(la2.convert_to_polygon().shapely_object) \
                    and la2.lanelet_id not in set(lanelets_not_allowed):
                        lanelet_ids.add(la1.lanelet_id)

    return  lanelet_ids

file_path = "./../tests/testScenarios/DEU_base_intersection.xml"
scenario, planning_problem_set = CommonRoadFileReader(file_path).open()

plt.figure(figsize=(25, 10))
rnd = MPRenderer()
scenario.lanelet_network.draw(rnd, draw_params=draw_params)
#rnd.render(show=True)

ids = extract_intersection(scenario.lanelet_network)
print(ids)
plt.show()