import math
import unittest

import crcpp
import numpy as np
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import Trajectory, TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.state import CustomState, InitialState

from ..predicate_test_utils import create_lanelet_network


class TestOrientationTowardsPredicate(unittest.TestCase):
    def setUp(self):
        self.lanelet_network = create_lanelet_network()

    def test_orientation_towards(self):
        # expected solutions
        exp_sol_monitor_mode_1 = False  # obs1 vehicle drives straight
        exp_sol_monitor_mode_2 = True  # obs1 vehicle drives to other from left
        exp_sol_monitor_mode_3 = False  # both drive straight
        exp_sol_monitor_mode_4 = True  # obs1 vehicle drives to other from right
        exp_sol_monitor_mode_5 = True  # obs1 drives to other from right

        obstacle_1 = DynamicObstacle(
            1,
            ObstacleType.CAR,
            Rectangle(5, 2),
            InitialState(
                time_step=0,
                position=np.array([10, 2]),
                velocity=10,
                acceleration=0,
                orientation=0,
            ),
            TrajectoryPrediction(
                Trajectory(
                    initial_time_step=1,
                    state_list=[
                        CustomState(
                            time_step=1,
                            position=np.array([20, 4]),
                            velocity=10,
                            acceleration=0,
                            orientation=-(1 / 5) * math.pi,
                        ),
                        CustomState(
                            time_step=2,
                            position=np.array([30, 6]),
                            velocity=10,
                            acceleration=0,
                            orientation=0,
                        ),
                        CustomState(
                            time_step=3,
                            position=np.array([40, 2]),
                            velocity=10,
                            acceleration=0,
                            orientation=(1 / 5) * math.pi,
                        ),
                    ],
                ),
                Rectangle(5, 2),
            ),
        )

        obstacle_2 = DynamicObstacle(
            2,
            ObstacleType.CAR,
            Rectangle(5, 2),
            InitialState(
                time_step=0,
                position=np.array([0, 2]),
                velocity=10,
                acceleration=0,
                orientation=0,
            ),
            TrajectoryPrediction(
                Trajectory(
                    initial_time_step=1,
                    state_list=[
                        CustomState(
                            time_step=1,
                            position=np.array([10, 2]),
                            velocity=10,
                            acceleration=0,
                            orientation=0,
                        ),
                        CustomState(
                            time_step=2,
                            position=np.array([20, 2]),
                            velocity=10,
                            acceleration=0,
                            orientation=0,
                        ),
                        CustomState(
                            time_step=3,
                            position=np.array([30, 4]),
                            velocity=10,
                            acceleration=0,
                            orientation=0,
                        ),
                    ],
                ),
                Rectangle(5, 2),
            ),
        )

        obstacle_3 = DynamicObstacle(
            3,
            ObstacleType.CAR,
            Rectangle(5, 2),
            InitialState(
                time_step=0,
                position=np.array([0, 12]),
                velocity=10,
                acceleration=0,
                orientation=0,
            ),
            TrajectoryPrediction(
                Trajectory(
                    initial_time_step=1,
                    state_list=[
                        CustomState(
                            time_step=1,
                            position=np.array([10, 12]),
                            velocity=10,
                            acceleration=0,
                            orientation=0,
                        ),
                        CustomState(
                            time_step=2,
                            position=np.array([20, 12]),
                            velocity=10,
                            acceleration=0,
                            orientation=0,
                        ),
                        CustomState(
                            time_step=3,
                            position=np.array([30, 12]),
                            velocity=10,
                            acceleration=0,
                            orientation=0,
                        ),
                    ],
                ),
                Rectangle(5, 2),
            ),
        )

        crcpp.World(
            "testScenario",
            0,
            0.1,
            "DEU",
            self.lanelet_network,
            [obstacle_1],
            [obstacle_2, obstacle_3],
        )

        # Monitor-Mode
        # sol_monitor_mode_1 = crcpp.orientation_towards_boolean_evaluation(123, 0, 1, 2)
        # sol_monitor_mode_2 = crcpp.orientation_towards_boolean_evaluation(123, 1, 1, 2)
        # sol_monitor_mode_3 = crcpp.orientation_towards_boolean_evaluation(123, 2, 1, 2)
        # sol_monitor_mode_4 = crcpp.orientation_towards_boolean_evaluation(123, 3, 1, 2)
        # sol_monitor_mode_5 = crcpp.orientation_towards_boolean_evaluation(123, 3, 1, 3)
        #
        # self.assertEqual(exp_sol_monitor_mode_1, sol_monitor_mode_1)
        # self.assertEqual(exp_sol_monitor_mode_2, sol_monitor_mode_2)
        # self.assertEqual(exp_sol_monitor_mode_3, sol_monitor_mode_3)
        # self.assertEqual(exp_sol_monitor_mode_4, sol_monitor_mode_4)
        # self.assertEqual(exp_sol_monitor_mode_5, sol_monitor_mode_5)
