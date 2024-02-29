import unittest
import numpy as np
import math
import os

import crcpp

from commonroad.scenario.state import CustomState, InitialState
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle
from commonroad.prediction.prediction import TrajectoryPrediction, Trajectory
from commonroad.common.file_reader import CommonRoadFileReader


class TestInIntersectionMainAreaPredicate(unittest.TestCase):
    def setUp(self):
        path = os.path.dirname(os.path.realpath(__file__))
        filename = path + '/../../../scenarios/predicates/DEU_TrafficLightTest-1_1_T-1.xml'
        scenario, _ = CommonRoadFileReader(filename).open()
        self.lanelet_network = scenario.lanelet_network
        self.dt = scenario.dt

    def test_in_intersection_main_area(self):
        # expected solutions
        exp_sol_monitor_mode_1 = False  # in front of intersection / completely on incoming
        exp_sol_monitor_mode_2 = True  # standing on stop line -> partially in intersection
        exp_sol_monitor_mode_3 = True  # inside intersection
        exp_sol_monitor_mode_4 = False  # left intersection

        obstacle_1 = DynamicObstacle(1, ObstacleType.CAR, Rectangle(5, 2),
                                     InitialState(time_step=0, position=np.array([26.5, -7.5]), velocity=0,
                                                  acceleration=0,
                                                  orientation=math.pi / 2),
                                     TrajectoryPrediction(Trajectory(initial_time_step=1, state_list=[
                                         CustomState(time_step=1, position=np.array([26.5, 3.5]), velocity=0,
                                                     acceleration=0, orientation=math.pi / 2)]), Rectangle(5, 2)))

        obstacle_2 = DynamicObstacle(2, ObstacleType.CAR, Rectangle(5, 2),
                                     InitialState(time_step=0, position=np.array([26.5, 3.0]), velocity=0,
                                                  acceleration=0, orientation=-math.pi),
                                     TrajectoryPrediction(Trajectory(initial_time_step=1, state_list=[
                                         CustomState(time_step=1, position=np.array([7.0, 3.0]), velocity=0,
                                                     acceleration=0,
                                                     orientation=-math.pi)]), Rectangle(5, 2)))

        crcpp.register_scenario(123, "testScenario", 0, self.dt, "DEU", self.lanelet_network, [obstacle_1, obstacle_2], [])

        # Monitor-Mode
        sol_monitor_mode_1 = crcpp.in_intersection_main_area_boolean_evaluation(123, 0, 1)
        sol_monitor_mode_2 = crcpp.in_intersection_main_area_boolean_evaluation(123, 1, 1)
        sol_monitor_mode_3 = crcpp.in_intersection_main_area_boolean_evaluation(123, 0, 2)
        sol_monitor_mode_4 = crcpp.in_intersection_main_area_boolean_evaluation(123, 1, 2)

        self.assertEqual(exp_sol_monitor_mode_1, sol_monitor_mode_1)
        self.assertEqual(exp_sol_monitor_mode_2, sol_monitor_mode_2)
        self.assertEqual(exp_sol_monitor_mode_3, sol_monitor_mode_3)
        self.assertEqual(exp_sol_monitor_mode_4, sol_monitor_mode_4)

        crcpp.remove_scenario(123)
