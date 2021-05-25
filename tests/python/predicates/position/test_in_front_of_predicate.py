import unittest
import numpy as np

import cpp_env_model

from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import State, ObstacleType, DynamicObstacle
from commonroad.prediction.prediction import TrajectoryPrediction, Trajectory
from ..predicate_test_utils import create_lanelet_network


class TestBrakingPredicates(unittest.TestCase):
    def setUp(self):
        self.lanelet_network = create_lanelet_network()

    def test_in_front_of(self):
        # expected solutions
        exp_sol_monitor_mode_1 = False  # ego vehicle behind
        exp_sol_monitor_mode_2 = False  # ego vehicle and other vehicle have same occupancy
        exp_sol_monitor_mode_3 = False  # ego vehicle is not completely in front
        exp_sol_monitor_mode_4 = True  # ego vehicle is in front in same lane
        exp_sol_monitor_mode_5 = True  # ego vehicle is in front in another lane
        exp_sol_robustness_mode_1 = -13.0
        exp_sol_robustness_mode_2 = -5.0
        exp_sol_robustness_mode_3 = -3.0
        exp_sol_robustness_mode_4 = 5.0
        exp_sol_robustness_mode_5 = 14.0

        obstacle_1 = DynamicObstacle(1, ObstacleType.CAR, Rectangle(5, 2),
                                     State(time_step=0, position=np.array([0, 0]), velocity=10, acceleration=0,
                                           orientation=0),
                                     TrajectoryPrediction(Trajectory(initial_time_step=1, state_list=[
                                         State(time_step=1, position=np.array([10, 0]), velocity=4,
                                               acceleration=-6, orientation=0),
                                         State(time_step=2, position=np.array([14, 0]), velocity=10,
                                               acceleration=6, orientation=0),
                                         State(time_step=3, position=np.array([24, 0]), velocity=5,
                                               acceleration=-5, orientation=0),
                                         State(time_step=4, position=np.array([29, 0]), velocity=5,
                                               acceleration=0, orientation=0)]), Rectangle(5, 2)))

        obstacle_2 = DynamicObstacle(2, ObstacleType.CAR, Rectangle(5, 2),
                                     State(time_step=0, position=np.array([8, 0]), velocity=2,
                                           acceleration=0, orientation=0),
                                     TrajectoryPrediction(Trajectory(initial_time_step=1, state_list=[
                                         State(time_step=1, position=np.array([10, 0]), velocity=2, acceleration=0,
                                               orientation=0),
                                         State(time_step=2, position=np.array([12, 0]), velocity=2, acceleration=0,
                                               orientation=0),
                                         State(time_step=3, position=np.array([14, 0]), velocity=2, acceleration=0,
                                               orientation=0)]), Rectangle(5, 2)))

        obstacle_3 = DynamicObstacle(3, ObstacleType.CAR, Rectangle(5, 2),
                                     State(time_step=3, position=np.array([0, 0]), velocity=10,
                                           acceleration=0, orientation=0),
                                     TrajectoryPrediction(Trajectory(initial_time_step=4, state_list=[
                                         State(time_step=4, position=np.array([10, 0]), velocity=10, acceleration=0,
                                               orientation=0)]), Rectangle(5, 2)))

        cpp_env_model.register_scenario(123, 0, self.lanelet_network, [obstacle_2, obstacle_3], [obstacle_1])

        # Monitor-Mode
        sol_monitor_mode_1 = cpp_env_model.in_front_of_boolean_evaluation(123, 0, 1, 2)
        sol_monitor_mode_2 = cpp_env_model.in_front_of_boolean_evaluation(123, 1, 1, 2)
        sol_monitor_mode_3 = cpp_env_model.in_front_of_boolean_evaluation(123, 2, 1, 2)
        sol_monitor_mode_4 = cpp_env_model.in_front_of_boolean_evaluation(123, 3, 1, 2)
        sol_monitor_mode_5 = cpp_env_model.in_front_of_boolean_evaluation(123, 4, 1, 3)

        self.assertEqual(exp_sol_monitor_mode_1, sol_monitor_mode_1)
        self.assertEqual(exp_sol_monitor_mode_2, sol_monitor_mode_2)
        self.assertEqual(exp_sol_monitor_mode_3, sol_monitor_mode_3)
        self.assertEqual(exp_sol_monitor_mode_4, sol_monitor_mode_4)
        self.assertEqual(exp_sol_monitor_mode_5, sol_monitor_mode_5)

        # # Constraint-Mode
        # TODO: not yet supported by Python interface

        # Robustness-Mode
        sol_robustness_mode_1 = cpp_env_model.in_front_of_robust_evaluation(123, 0, 1, 2)
        sol_robustness_mode_2 = cpp_env_model.in_front_of_robust_evaluation(123, 1, 1, 2)
        sol_robustness_mode_3 = cpp_env_model.in_front_of_robust_evaluation(123, 2, 1, 2)
        sol_robustness_mode_4 = cpp_env_model.in_front_of_robust_evaluation(123, 3, 1, 2)
        sol_robustness_mode_5 = cpp_env_model.in_front_of_robust_evaluation(123, 4, 1, 3)

        self.assertEqual(exp_sol_robustness_mode_1, sol_robustness_mode_1)
        self.assertEqual(exp_sol_robustness_mode_2, sol_robustness_mode_2)
        self.assertEqual(exp_sol_robustness_mode_3, sol_robustness_mode_3)
        self.assertEqual(exp_sol_robustness_mode_4, sol_robustness_mode_4)
        self.assertEqual(exp_sol_robustness_mode_5, sol_robustness_mode_5)

        cpp_env_model.remove_scenario(123)
