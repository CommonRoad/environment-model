# import unittest
# import numpy as np
#
# import cpp_env_model
#
# from commonroad.geometry.shape import Rectangle
# from commonroad.scenario.lanelet import LaneletNetwork, Lanelet, LineMarking, LaneletType
# from commonroad.scenario.obstacle import State, ObstacleType, DynamicObstacle
# from commonroad.prediction.prediction import TrajectoryPrediction, Trajectory
#
#
# class TestBrakingPredicates(unittest.TestCase):
#     def setUp(self):
#         right_vertices_lane_1 = np.array([[0, 0], [10, 0], [20, 0], [30, 0], [40, 0], [50, 0], [60, 0], [70, 0],
#                                           [80, 1], [90, 0], [100, 0], [110, 0]])
#         left_vertices_lane_1 = np.array([[0, 4], [10, 4], [20, 4], [30, 4], [40, 4], [50, 4], [60, 4], [70, 4],
#                                          [80, 1], [90, 4], [100, 4], [110, 4]])
#         center_vertices_lane_1 = np.array([[0, 2], [10, 2], [20, 2], [30, 2], [40, 2], [50, 2], [60, 2], [70, 2],
#                                            [80, 1], [90, 2], [100, 2], [110, 2]])
#         lanelet_1 = Lanelet(left_vertices_lane_1, center_vertices_lane_1, right_vertices_lane_1, lanelet_id=1,
#                             adjacent_left=2, adjacent_left_same_direction=True,
#                             lanelet_type={LaneletType.INTERSTATE, LaneletType.SHOULDER})
#
#         right_vertices_lane_2 = np.array([[0, 4], [10, 4], [20, 4], [30, 4], [40, 4], [50, 4], [60, 4], [70, 4],
#                                           [80, 4], [90, 4], [100, 4], [110, 4]])
#         left_vertices_lane_2 = np.array([[0, 8], [10, 8], [20, 8], [30, 8], [40, 8], [50, 8], [60, 8], [70, 8],
#                                          [80, 8], [90, 8], [100, 8], [110, 8]])
#         center_vertices_lane_2 = np.array([[0, 6], [10, 6], [20, 6], [30, 6], [40, 6], [50, 6], [60, 6],
#                                            [70, 6], [80, 6], [90, 6], [100, 6], [110, 6]])
#         lanelet_2 = Lanelet(left_vertices_lane_2, center_vertices_lane_2, right_vertices_lane_2, lanelet_id=2,
#                             adjacent_left=3, adjacent_left_same_direction=True,
#                             adjacent_right=1, adjacent_right_same_direction=True,
#                             lanelet_type={LaneletType.INTERSTATE},
#                             line_marking_left_vertices=LineMarking.BROAD_DASHED)
#
#         right_vertices_lane_3 = np.array([[0, 8], [10, 8], [20, 8], [30, 8], [40, 8], [50, 8], [60, 8], [70, 8],
#                                           [80, 8], [90, 8], [100, 8], [110, 8]])
#         left_vertices_lane_3 = np.array([[0, 12], [10, 12], [20, 12], [30, 12], [40, 12], [50, 12], [60, 12], [70, 12],
#                                          [80, 12], [90, 12], [100, 12], [110, 12]])
#         center_vertices_lane_3 = np.array([[0, 10], [10, 10], [20, 10], [30, 10], [40, 10], [50, 10], [60, 10],
#                                            [70, 10], [80, 10], [90, 10], [100, 10], [110, 10]])
#         lanelet_3 = Lanelet(left_vertices_lane_3, center_vertices_lane_3, right_vertices_lane_3, lanelet_id=3,
#                             adjacent_left=4, adjacent_left_same_direction=True,
#                             adjacent_right=2, adjacent_right_same_direction=True,
#                             lanelet_type={LaneletType.INTERSTATE, LaneletType.MAIN_CARRIAGE_WAY},
#                             line_marking_right_vertices=LineMarking.BROAD_DASHED)
#
#         right_vertices_lane_4 = np.array([[0, 12], [10, 12], [20, 12], [30, 12], [40, 12], [50, 12], [60, 12], [70, 12],
#                                           [80, 12], [90, 12], [100, 12], [110, 12]])
#         left_vertices_lane_4 = np.array([[0, 16], [10, 16], [20, 16], [30, 16], [40, 16], [50, 16], [60, 16], [70, 16],
#                                          [80, 16], [90, 16], [100, 16], [110, 16]])
#         center_vertices_lane_4 = np.array([[0, 14], [10, 14], [20, 14], [30, 14], [40, 14], [50, 14], [60, 14],
#                                            [70, 14], [80, 14], [90, 14], [100, 14], [110, 14]])
#         lanelet_4 = Lanelet(left_vertices_lane_4, center_vertices_lane_4, right_vertices_lane_4, lanelet_id=4,
#                             adjacent_left=5, adjacent_left_same_direction=True,
#                             adjacent_right=3, adjacent_right_same_direction=True,
#                             lanelet_type={LaneletType.INTERSTATE, LaneletType.MAIN_CARRIAGE_WAY})
#
#         right_vertices_lane_5 = np.array([[0, 16], [10, 16], [20, 16], [30, 16], [40, 16], [50, 16], [60, 16], [70, 16],
#                                           [80, 16], [90, 16], [100, 16], [110, 16]])
#         left_vertices_lane_5 = np.array([[0, 20], [10, 20], [20, 20], [30, 20], [40, 20], [50, 20], [60, 20], [70, 20],
#                                          [80, 20], [90, 20], [100, 20], [110, 20]])
#         center_vertices_lane_5 = np.array([[0, 18], [10, 18], [20, 18], [30, 18], [40, 18], [50, 18], [60, 18],
#                                            [70, 18], [80, 18], [90, 18], [00, 18], [110, 18]])
#         lanelet_5 = Lanelet(left_vertices_lane_5, center_vertices_lane_5, right_vertices_lane_5, lanelet_id=5,
#                             adjacent_right=4, adjacent_right_same_direction=True,
#                             lanelet_type={LaneletType.INTERSTATE, LaneletType.ACCESS_RAMP})
#         self.lanelet_network = LaneletNetwork()
#         self.lanelet_network.add_lanelet(lanelet_1)
#         self.lanelet_network.add_lanelet(lanelet_2)
#         self.lanelet_network.add_lanelet(lanelet_3)
#         self.lanelet_network.add_lanelet(lanelet_4)
#         self.lanelet_network.add_lanelet(lanelet_5)
#
#     def test_in_front_of(self):
#         # expected solutions
#         exp_sol_monitor_mode_1 = False  # ego vehicle behind
#         exp_sol_monitor_mode_2 = False  # ego vehicle and other vehicle have same occupancy
#         exp_sol_monitor_mode_3 = False  # ego vehicle is not completely in front
#         exp_sol_monitor_mode_4 = True  # ego vehicle is in front in same lane
#         exp_sol_monitor_mode_5 = True  # ego vehicle is in front in another lane
#         exp_sol_constraint_mode_1 = 10.5
#         exp_sol_constraint_mode_2 = 12.5
#         exp_sol_constraint_mode_3 = 14.5
#         exp_sol_constraint_mode_4 = 16.5
#         exp_sol_constraint_mode_5 = 12.5
#         exp_sol_robustness_mode_1 = -13.0
#         exp_sol_robustness_mode_2 = -5.0
#         exp_sol_robustness_mode_3 = -3.0
#         exp_sol_robustness_mode_4 = 5.0
#         exp_sol_robustness_mode_5 = 14.0
#
#         obstacle_1 = DynamicObstacle(1, ObstacleType.CAR, Rectangle(5, 2),
#                                      State(time_step=0, position=np.array([0, 0]), velocity=10, acceleration=0,
#                                            orientation=0),
#                                      TrajectoryPrediction(Trajectory(initial_time_step=1, state_list=[
#                                          State(time_step=1, position=np.array([10, 0]), velocity=4,
#                                                acceleration=-6, orientation=0),
#                                          State(time_step=2, position=np.array([14, 0]), velocity=10,
#                                                acceleration=6, orientation=0),
#                                          State(time_step=3, position=np.array([24, 0]), velocity=5,
#                                                acceleration=-5, orientation=0),
#                                          State(time_step=4, position=np.array([29, 0]), velocity=5,
#                                                acceleration=0, orientation=0)]), Rectangle(5, 2)))
#
#         obstacle_2 = DynamicObstacle(2, ObstacleType.CAR, Rectangle(5, 2),
#                                      State(time_step=0, position=np.array([8, 0]), velocity=2,
#                                            acceleration=0, orientation=0),
#                                      TrajectoryPrediction(Trajectory(initial_time_step=1, state_list=[
#                                          State(time_step=1, position=np.array([10, 0]), velocity=2, acceleration=0,
#                                                orientation=0),
#                                          State(time_step=2, position=np.array([12, 0]), velocity=2, acceleration=0,
#                                                orientation=0),
#                                          State(time_step=3, position=np.array([14, 0]), velocity=2, acceleration=0,
#                                                orientation=0)]), Rectangle(5, 2)))
#
#         obstacle_3 = DynamicObstacle(3, ObstacleType.CAR, Rectangle(5, 2),
#                                      State(time_step=3, position=np.array([0, 0]), velocity=10,
#                                            acceleration=0, orientation=0),
#                                      TrajectoryPrediction(Trajectory(initial_time_step=4, state_list=[
#                                          State(time_step=4, position=np.array([10, 0]), velocity=10, acceleration=0,
#                                                orientation=0)]), Rectangle(5, 2)))
#
#         cpp_env_model.register_scenario(123, 0, self.lanelet_network, [obstacle_2, obstacle_3], [obstacle_1])
#
#         # Monitor-Mode
#         sol_monitor_mode_1 = cpp_env_model.in_front_of_boolean_evaluation(123, 0, 1, [2])
#         sol_monitor_mode_2 = cpp_env_model.in_front_of_boolean_evaluation(123, 1, 1, [2])
#         sol_monitor_mode_3 = cpp_env_model.in_front_of_boolean_evaluation(123, 2, 1, [2])
#         sol_monitor_mode_4 = cpp_env_model.in_front_of_boolean_evaluation(123, 3, 1, [2])
#         sol_monitor_mode_5 = cpp_env_model.in_front_of_boolean_evaluation(123, 4, 1, [3])
#
#         self.assertEqual(exp_sol_monitor_mode_1, sol_monitor_mode_1)
#         self.assertEqual(exp_sol_monitor_mode_2, sol_monitor_mode_2)
#     #    self.assertEqual(exp_sol_monitor_mode_3, sol_monitor_mode_3)
#     #    self.assertEqual(exp_sol_monitor_mode_4, sol_monitor_mode_4)
#         self.assertEqual(exp_sol_monitor_mode_5, sol_monitor_mode_5)
#
#         # # Constraint-Mode
#         # not yet supported by Python interface
#
#         # Robustness-Mode
#         sol_robustness_mode_1 = cpp_env_model.in_front_of_robust_evaluation(123, 0, 1, [2])
#         sol_robustness_mode_2 = cpp_env_model.in_front_of_robust_evaluation(123, 1, 1, [2])
#         sol_robustness_mode_3 = cpp_env_model.in_front_of_robust_evaluation(123, 2, 1, [2])
#         sol_robustness_mode_4 = cpp_env_model.in_front_of_robust_evaluation(123, 3, 1, [2])
#         sol_robustness_mode_5 = cpp_env_model.in_front_of_robust_evaluation(123, 4, 1, [3])
#
#         self.assertEqual(exp_sol_robustness_mode_1, sol_robustness_mode_1)
#         self.assertEqual(exp_sol_robustness_mode_2, sol_robustness_mode_2)
#         self.assertEqual(exp_sol_robustness_mode_3, sol_robustness_mode_3)
#         self.assertEqual(exp_sol_robustness_mode_4, sol_robustness_mode_4)
#         self.assertEqual(exp_sol_robustness_mode_5, sol_robustness_mode_5)
