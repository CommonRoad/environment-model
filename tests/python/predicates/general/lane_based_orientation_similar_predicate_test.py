import unittest
import numpy as np
import math

import cpp_env_model

from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import State, ObstacleType, DynamicObstacle
from commonroad.prediction.prediction import TrajectoryPrediction, Trajectory
from ..predicate_test_utils import create_lanelet_network


class TestOrientationTowardsPredicate(unittest.TestCase):
    def setUp(self):
        self.lanelet_network = create_lanelet_network()

    def test_orientation_towards(self):
        # expected solutions
        exp_sol_monitor_mode_1 = True  # ego vehicle drives straight
        exp_sol_monitor_mode_2 = False  # ego vehicle drives to other from left
        exp_sol_monitor_mode_3 = True  # both drive straight
        exp_sol_monitor_mode_4 = False  # ego vehicle drives to other from right
        exp_sol_monitor_mode_5 = False  # ego vehicle drives to other from right

        obstacle_1 = DynamicObstacle(1, ObstacleType.CAR, Rectangle(5, 2),
                                     State(time_step=0, position=np.array([10, 2]), velocity=10, acceleration=0,
                                           orientation=0),
                                     TrajectoryPrediction(Trajectory(initial_time_step=1, state_list=[
                                         State(time_step=1, position=np.array([20, 4]), velocity=10,
                                               acceleration=0, orientation=(1 / 5) * math.pi),
                                         State(time_step=2, position=np.array([30, 6]), velocity=10,
                                               acceleration=0, orientation=0),
                                         State(time_step=3, position=np.array([40, 4]), velocity=10,
                                               acceleration=0, orientation=-(1 / 5) * math.pi)]), Rectangle(5, 2)))

        obstacle_2 = DynamicObstacle(2, ObstacleType.CAR, Rectangle(5, 2),
                                     State(time_step=0, position=np.array([0, 2]), velocity=10,
                                           acceleration=0, orientation=0),
                                     TrajectoryPrediction(Trajectory(initial_time_step=1, state_list=[
                                         State(time_step=1, position=np.array([10, 4.01]), velocity=10, acceleration=0,
                                               orientation=0),
                                         State(time_step=2, position=np.array([20, 2]), velocity=10, acceleration=0,
                                               orientation=0),
                                         State(time_step=3, position=np.array([30, 2]), velocity=10, acceleration=0,
                                               orientation=0)]), Rectangle(5, 2)))

        obstacle_3 = DynamicObstacle(3, ObstacleType.CAR, Rectangle(5, 2),
                                     State(time_step=1, position=np.array([0, 12]), velocity=10,
                                           acceleration=0, orientation=0),
                                     TrajectoryPrediction(Trajectory(initial_time_step=1, state_list=[
                                         State(time_step=1, position=np.array([0, 12]), velocity=10, acceleration=0,
                                               orientation=0)]), Rectangle(5, 2)))

        cpp_env_model.register_scenario(123, 0, 0.1, "DEU", self.lanelet_network, [obstacle_1],
                                        [obstacle_2, obstacle_3])

        # Monitor-Mode
        sol_monitor_mode_1 = cpp_env_model.lane_based_orientation_similar_boolean_evaluation(123, 0, 1, 2)
        sol_monitor_mode_2 = cpp_env_model.lane_based_orientation_similar_boolean_evaluation(123, 1, 1, 2)
        sol_monitor_mode_3 = cpp_env_model.lane_based_orientation_similar_boolean_evaluation(123, 2, 1, 2)
        sol_monitor_mode_4 = cpp_env_model.lane_based_orientation_similar_boolean_evaluation(123, 3, 1, 2)
        sol_monitor_mode_5 = cpp_env_model.lane_based_orientation_similar_boolean_evaluation(123, 1, 1, 3)

        self.assertEqual(exp_sol_monitor_mode_1, sol_monitor_mode_1)
        self.assertEqual(exp_sol_monitor_mode_2, sol_monitor_mode_2)
        self.assertEqual(exp_sol_monitor_mode_3, sol_monitor_mode_3)
        self.assertEqual(exp_sol_monitor_mode_4, sol_monitor_mode_4)
        self.assertEqual(exp_sol_monitor_mode_5, sol_monitor_mode_5)

        cpp_env_model.remove_scenario(123)
