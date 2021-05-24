import unittest
import numpy as np

import cpp_env_model

from commonroad.geometry.shape import Rectangle
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet
from commonroad.scenario.obstacle import State, ObstacleType, DynamicObstacle
from commonroad.prediction.prediction import TrajectoryPrediction, Trajectory


class TestBrakingPredicates(unittest.TestCase):
    def setUp(self):
        right_vertices = np.array([[0, 0], [10, 0], [20, 0], [30, 0], [40, 0], [50, 0], [60, 0], [70, 0], [80, 0]])
        left_vertices = np.array([[0, 4], [10, 4], [20, 4], [30, 4], [40, 4], [50, 4], [60, 4], [70, 4], [80, 4]])
        center_vertices = np.array([[0, 2], [10, 2], [20, 2], [30, 2], [40, 2], [50, 2], [60, 2], [70, 2], [80, 2]])
        lanelet_id = 1
        self.lanelet_network = LaneletNetwork()
        self.lanelet_network.add_lanelet(Lanelet(left_vertices, center_vertices, right_vertices, lanelet_id))

    def test_unnecessary_braking(self):
        a_abrupt = -2

        # expected solutions
        exp_sol_monitor_mode_1 = False  # a_ego > 0
        exp_sol_monitor_mode_2 = True  # a_ego < a_lead - |a_abrupt| for single leading vehicle
        exp_sol_monitor_mode_3 = True  # a_ego < a_lead - |a_abrupt| for all leading vehicles
        exp_sol_monitor_mode_4 = False  # a_ego > a_lead - |a_abrupt| for all leading vehicles
        exp_sol_monitor_mode_5 = True  # a_ego < a_abrupt; no leading vehicle
        exp_sol_monitor_mode_6 = False  # a_ego > a_abrupt; no leading vehicle

        exp_sol_robustness_mode_1 = a_abrupt
        exp_sol_robustness_mode_2 = 4 + a_abrupt
        exp_sol_robustness_mode_3 = 5 + a_abrupt
        exp_sol_robustness_mode_4 = 1.5 + a_abrupt
        exp_sol_robustness_mode_5 = 8 + a_abrupt
        exp_sol_robustness_mode_6 = -2 + a_abrupt

        obstacle_1 = DynamicObstacle(1, ObstacleType.CAR, Rectangle(5, 2),
                                     State(time_step=0, position=np.array([0, 0]), velocity=10, acceleration=1,
                                           orientation=0),
                                     TrajectoryPrediction(Trajectory(initial_time_step=1, state_list=[
                                         State(time_step=1, position=np.array([10, 0]), velocity=10,
                                               acceleration=-5, orientation=0),
                                         State(time_step=2, position=np.array([20, 0]), velocity=10,
                                               acceleration=-7, orientation=0),
                                         State(time_step=3, position=np.array([30, 0]), velocity=10,
                                               acceleration=-3, orientation=0),
                                         State(time_step=4, position=np.array([40, 0]), velocity=10,
                                               acceleration=-8, orientation=0),
                                         State(time_step=5, position=np.array([50, 0]), velocity=10,
                                               acceleration=2, orientation=0)]), Rectangle(5, 2)))

        obstacle_2 = DynamicObstacle(2, ObstacleType.CAR, Rectangle(5, 2),
                                     State(time_step=0, position=np.array([10, 0]), velocity=10, acceleration=1,
                                           orientation=0),
                                     TrajectoryPrediction(Trajectory(initial_time_step=1, state_list=[
                                         State(time_step=1, position=np.array([20, 0]), velocity=10, acceleration=-4,
                                               orientation=0),
                                         State(time_step=2, position=np.array([30, 0]), velocity=10, acceleration=-2,
                                               orientation=0),
                                         State(time_step=3, position=np.array([40, 0]), velocity=10, acceleration=-1.5,
                                               orientation=0)]), Rectangle(5, 2)))

        obstacle_3 = DynamicObstacle(2, ObstacleType.CAR, Rectangle(5, 2),
                                     State(time_step=0, position=np.array([20, 0]), velocity=10,
                                           acceleration=1, orientation=0),
                                     TrajectoryPrediction(Trajectory(initial_time_step=1, state_list=[
                                         State(time_step=1, position=np.array([30, 0]), velocity=10, acceleration=-1,
                                               orientation=0),
                                         State(time_step=2, position=np.array([40, 0]), velocity=10, acceleration=-3,
                                               orientation=0),
                                         State(time_step=3, position=np.array([50, 0]), velocity=10, acceleration=-2,
                                               orientation=0)]), Rectangle(5, 2)))

        cpp_env_model.register_scenario(123, 0, self.lanelet_network, [obstacle_2, obstacle_3], [obstacle_1])

        # Monitor-Mode
        sol_monitor_mode_1 = cpp_env_model.unnecessary_braking_boolean_evaluation(123, 0, 1, [2, 3])
        sol_monitor_mode_2 = cpp_env_model.unnecessary_braking_boolean_evaluation(123, 1, 1, [2, 3])
        sol_monitor_mode_3 = cpp_env_model.unnecessary_braking_boolean_evaluation(123, 2, 1, [2, 3])
        sol_monitor_mode_4 = cpp_env_model.unnecessary_braking_boolean_evaluation(123, 3, 1, [2, 3])
        sol_monitor_mode_5 = cpp_env_model.unnecessary_braking_boolean_evaluation(123, 4, 1, [2, 3])
        sol_monitor_mode_6 = cpp_env_model.unnecessary_braking_boolean_evaluation(123, 5, 1, [2, 3])
        self.assertEqual(exp_sol_monitor_mode_1, sol_monitor_mode_1)
        self.assertEqual(exp_sol_monitor_mode_2, sol_monitor_mode_2)
        self.assertEqual(exp_sol_monitor_mode_3, sol_monitor_mode_3)
        self.assertEqual(exp_sol_monitor_mode_4, sol_monitor_mode_4)
        self.assertEqual(exp_sol_monitor_mode_5, sol_monitor_mode_5)
        self.assertEqual(exp_sol_monitor_mode_6, sol_monitor_mode_6)

        # Constraint-Mode
        # TODO: not yet supported by Python interface

        # Robustness-Mode
        sol_robustness_mode_1 = cpp_env_model.unnecessary_braking_robust_evaluation(123, 0, 1, [2, 3])
        sol_robustness_mode_2 = cpp_env_model.unnecessary_braking_robust_evaluation(123, 1, 1, [2, 3])
        sol_robustness_mode_3 = cpp_env_model.unnecessary_braking_robust_evaluation(123, 2, 1, [2, 3])
        sol_robustness_mode_4 = cpp_env_model.unnecessary_braking_robust_evaluation(123, 3, 1, [2, 3])
        sol_robustness_mode_5 = cpp_env_model.unnecessary_braking_robust_evaluation(123, 4, 1, [2, 3])
        sol_robustness_mode_6 = cpp_env_model.unnecessary_braking_robust_evaluation(123, 5, 1, [2, 3])
        self.assertEqual(exp_sol_robustness_mode_1, sol_robustness_mode_1)
        self.assertEqual(exp_sol_robustness_mode_2, sol_robustness_mode_2)
        self.assertEqual(exp_sol_robustness_mode_3, sol_robustness_mode_3)
        self.assertEqual(exp_sol_robustness_mode_4, sol_robustness_mode_4)
        self.assertEqual(exp_sol_robustness_mode_5, sol_robustness_mode_5)
        self.assertEqual(exp_sol_robustness_mode_6, sol_robustness_mode_6)

        cpp_env_model.remove_scenario(123)
