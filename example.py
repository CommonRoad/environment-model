import numpy as np
import math
import os

import cpp_env_model

from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import State, ObstacleType, DynamicObstacle
from commonroad.prediction.prediction import TrajectoryPrediction, Trajectory
from commonroad.common.file_reader import CommonRoadFileReader

path = os.path.dirname(os.path.realpath(__file__))
filename = path + '/tests/scenarios/DEU_TrafficLightTest-1_1_T-1.xml'
scenario, _ = CommonRoadFileReader(filename).open()
lanelet_network = scenario.lanelet_network

# expected solutions
exp_sol_monitor_mode_1 = True  # in front of intersection/traffic light -> completely on incoming
exp_sol_monitor_mode_2 = True  # standing on stop line -> partially in intersection
exp_sol_monitor_mode_3 = False  # inside intersection
exp_sol_monitor_mode_4 = False  # left intersection

obstacle_1 = DynamicObstacle(1, ObstacleType.CAR, Rectangle(5, 2),
                             State(time_step=0, position=np.array([26.5, -7.5]), velocity=0, acceleration=0,
                                   orientation=math.pi/2),
                             TrajectoryPrediction(Trajectory(initial_time_step=1, state_list=[
                                 State(time_step=1, position=np.array([26.5, 3.5]), velocity=0,
                                       acceleration=0, orientation=math.pi/2)]), Rectangle(5, 2)))

obstacle_2 = DynamicObstacle(2, ObstacleType.CAR, Rectangle(5, 2),
                             State(time_step=0, position=np.array([26.5, 3.0]), velocity=0,
                                   acceleration=0, orientation=-math.pi),
                             TrajectoryPrediction(Trajectory(initial_time_step=1, state_list=[
                                 State(time_step=1, position=np.array([7.0, 3.0]), velocity=0, acceleration=0,
                                       orientation=-math.pi)]), Rectangle(5, 2)))

cpp_env_model.register_scenario(123, 0, "DEU", lanelet_network, [obstacle_1, obstacle_2], [])

# Monitor-Mode
print(cpp_env_model.at_red_right_traffic_light_boolean_evaluation(123, 0, 1))

cpp_env_model.remove_scenario(123)