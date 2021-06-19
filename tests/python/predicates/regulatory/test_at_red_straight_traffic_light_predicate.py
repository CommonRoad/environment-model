import unittest
import numpy as np
import math
import os

import cpp_env_model

from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import State, ObstacleType, DynamicObstacle
from commonroad.prediction.prediction import TrajectoryPrediction, Trajectory
from commonroad.common.file_reader import CommonRoadFileReader


class TestAtRedStraightTrafficLightPredicates(unittest.TestCase):
    def setUp(self):
        path = os.path.dirname(os.path.realpath(__file__))
        filename = path + '/../../../scenarios/DEU_TrafficLightTest-1_1_T-1.xml'
        scenario, _ = CommonRoadFileReader(filename).open()
        self.lanelet_network = scenario.lanelet_network

    def test_at_red_straight_traffic_light(self):
        # expected solutions
        exp_sol_monitor_mode_1 = True  # in front of intersection/traffic light -> completely on incoming
        exp_sol_monitor_mode_2 = True  # standing on stop line -> partially in intersection
        exp_sol_monitor_mode_3 = False  # inside intersection
        exp_sol_monitor_mode_4 = False  # left intersection
        exp_sol_monitor_mode_5 = False  # in front of intersection/traffic light with
        # another direction -> completely on incoming
        exp_sol_monitor_mode_6 = False  # standing on stop line -> partially in
        # intersection, traffic light has another direction

        obstacle_1 = DynamicObstacle(1, ObstacleType.CAR, Rectangle(5, 2),
                                     State(time_step=0, position=np.array([40.0, 3.0]), velocity=0, acceleration=0,
                                           orientation=-math.pi),
                                     TrajectoryPrediction(Trajectory(initial_time_step=1, state_list=[
                                         State(time_step=1, position=np.array([30.0, 3.0]), velocity=0,
                                               acceleration=0, orientation=-math.pi),
                                         State(time_step=2, position=np.array([25.0, 3.0]), velocity=0,
                                               acceleration=0, orientation=-math.pi),
                                         State(time_step=3, position=np.array([10.0, 3.0]), velocity=0,
                                               acceleration=0, orientation=-math.pi)]), Rectangle(5, 2)))

        obstacle_2 = DynamicObstacle(2, ObstacleType.CAR, Rectangle(5, 2),
                                     State(time_step=0, position=np.array([10.0, 0.0]), velocity=0,
                                           acceleration=0, orientation=0),
                                     TrajectoryPrediction(Trajectory(initial_time_step=1, state_list=[
                                         State(time_step=1, position=np.array([20.0, 0.0]), velocity=0, acceleration=0,
                                               orientation=0)]), Rectangle(5, 2)))

        cpp_env_model.register_scenario(123, 0, "DEU", self.lanelet_network, [obstacle_1, obstacle_2], [])

        # Monitor-Mode
        sol_monitor_mode_1 = cpp_env_model.at_red_straight_traffic_light_boolean_evaluation(123, 0, 1)
        sol_monitor_mode_2 = cpp_env_model.at_red_straight_traffic_light_boolean_evaluation(123, 1, 1)
        sol_monitor_mode_3 = cpp_env_model.at_red_straight_traffic_light_boolean_evaluation(123, 2, 1)
        sol_monitor_mode_4 = cpp_env_model.at_red_straight_traffic_light_boolean_evaluation(123, 3, 1)
        sol_monitor_mode_5 = cpp_env_model.at_red_straight_traffic_light_boolean_evaluation(123, 0, 2)
        sol_monitor_mode_6 = cpp_env_model.at_red_straight_traffic_light_boolean_evaluation(123, 1, 2)

        self.assertEqual(exp_sol_monitor_mode_1, sol_monitor_mode_1)
        self.assertEqual(exp_sol_monitor_mode_2, sol_monitor_mode_2)
        self.assertEqual(exp_sol_monitor_mode_3, sol_monitor_mode_3)
        self.assertEqual(exp_sol_monitor_mode_4, sol_monitor_mode_4)
        self.assertEqual(exp_sol_monitor_mode_5, sol_monitor_mode_5)
        self.assertEqual(exp_sol_monitor_mode_6, sol_monitor_mode_6)

        cpp_env_model.remove_scenario(123)
