import unittest
import numpy as np

import cpp_env_model

from commonroad.geometry.shape import Rectangle
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet
from commonroad.scenario.obstacle import State, ObstacleType, DynamicObstacle
from commonroad.prediction.prediction import TrajectoryPrediction, Trajectory


class TestSafeDistancePredicate(unittest.TestCase):
    def setUp(self):
        right_vertices = np.array([[0, 0], [10, 0], [20, 0], [30, 0], [40, 0], [50, 0], [60, 0], [70, 0], [80, 0]])
        left_vertices = np.array([[0, 4], [10, 4], [20, 4], [30, 4], [40, 4], [50, 4], [60, 4], [70, 4], [80, 4]])
        center_vertices = np.array([[0, 2], [10, 2], [20, 2], [30, 2], [40, 2], [50, 2], [60, 2], [70, 2], [80, 2]])
        lanelet_id = 1
        self.lanelet_network = LaneletNetwork()
        self.lanelet_network.add_lanelet(Lanelet(left_vertices, center_vertices, right_vertices, lanelet_id))

    def test_keeps_safe_distance(self):
        # expected solutions
        exp_sol_monitor_mode_1 = True
        exp_sol_monitor_mode_2 = False
        exp_sol_robustness_mode_1 = 9.0
        exp_sol_robustness_mode_2 = -21.0

        ego_obstacle = DynamicObstacle(1, ObstacleType.CAR, Rectangle(5, 2),
                                       State(time_step=0, position=np.array([0, 0]), velocity=20, acceleration=-1,
                                             orientation=0),
                                       TrajectoryPrediction(Trajectory(initial_time_step=1, state_list=[
                                           State(time_step=1, position=np.array([20, 0]), velocity=20,
                                                 acceleration=-1, orientation=0)]), Rectangle(5, 2)))
        other_obstacle = DynamicObstacle(2, ObstacleType.CAR, Rectangle(5, 2),
                                         State(time_step=0, position=np.array([20, 0]), velocity=20,
                                               acceleration=-1, orientation=0),
                                         TrajectoryPrediction(Trajectory(initial_time_step=1, state_list=[
                                             State(time_step=1, position=np.array([30, 0]), velocity=0, acceleration=0,
                                                   orientation=0)]), Rectangle(5, 2)))

        cpp_env_model.register_scenario(123, 0, "DEU", self.lanelet_network, [ego_obstacle], [other_obstacle])

        # Monitor-Mode
        sol_monitor_mode_1_obstacles = cpp_env_model.safe_distance_boolean_evaluation(123, 0, 1, 2)
        sol_monitor_mode_2_obstacles = cpp_env_model.safe_distance_boolean_evaluation(123, 1, 1, 2)
        sol_monitor_mode_1_parameters = \
            cpp_env_model.safe_distance_boolean_evaluation(0, 20, 20, 20, -10.0, -10.0, 0.3, 5, 5)
        sol_monitor_mode_2_parameters = \
            cpp_env_model.safe_distance_boolean_evaluation(20, 30, 20, 0, -10.0, -10.0, 0.3, 5, 5)

        self.assertEqual(exp_sol_monitor_mode_1, sol_monitor_mode_1_obstacles)
        self.assertEqual(exp_sol_monitor_mode_2, sol_monitor_mode_2_obstacles)
        self.assertEqual(exp_sol_monitor_mode_1, sol_monitor_mode_1_parameters)
        self.assertEqual(exp_sol_monitor_mode_2, sol_monitor_mode_2_parameters)

        # Constraint-Mode
        # TODO: not yet supported by Python interface

        # # Robustness-Mode
        sol_robustness_mode_1_obstacles = cpp_env_model.safe_distance_robust_evaluation(123, 0, 1, 2)
        sol_robustness_mode_2_obstacles = cpp_env_model.safe_distance_robust_evaluation(123, 1, 1, 2)
        sol_robustness_mode_1_parameters = \
            cpp_env_model.safe_distance_robust_evaluation(0, 20, 20, 20, -10.0, -10.0, 0.3, 5, 5)
        sol_robustness_mode_2_parameters = \
            cpp_env_model.safe_distance_robust_evaluation(20, 30, 20, 0, -10.0, -10.0, 0.3, 5, 5)

        self.assertEqual(exp_sol_robustness_mode_1, sol_robustness_mode_1_obstacles)
        self.assertEqual(exp_sol_robustness_mode_2, sol_robustness_mode_2_obstacles)
        self.assertEqual(exp_sol_robustness_mode_1, sol_robustness_mode_1_parameters)
        self.assertEqual(exp_sol_robustness_mode_2, sol_robustness_mode_2_parameters)

        cpp_env_model.remove_scenario(123)

    def test_safe_distance(self):
        # invalid parameters a_min_follow, a_min_lead
        self.assertRaises(RuntimeError, cpp_env_model.safe_distance, 0, 0, -1, 0, 0)
        self.assertRaises(RuntimeError, cpp_env_model.safe_distance, 0, 0, 0, -1, 0)

        exp_sol = 0  # both vehicles standing, no reaction time
        solution = cpp_env_model.safe_distance(0, 0, -10, -10, 0)
        self.assertEqual(exp_sol, solution)

        exp_sol = 0  # both vehicles same velocity, no reaction time
        solution = cpp_env_model.safe_distance(5, 5, -10, -10, 0)
        self.assertEqual(exp_sol, solution)

        exp_sol = 50.0  # both vehicles same velocity, with reaction time
        solution = cpp_env_model.safe_distance(5, 5, -10, -10, 10)
        self.assertEqual(exp_sol, solution)

        exp_sol = 5.0  # following vehicle higher velocity, no reaction time
        solution = cpp_env_model.safe_distance(10, 0, -10, -10, 0)
        self.assertEqual(exp_sol, solution)

        exp_sol = -5.0  # leading vehicle higher velocity, no reaction time
        solution = cpp_env_model.safe_distance(0, 10, -10, -10, 0)
        self.assertEqual(exp_sol, solution)
