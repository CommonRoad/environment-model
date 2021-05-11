import numpy as np
import cpp_env_model

from commonroad.geometry.shape import Rectangle
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet
from commonroad.scenario.obstacle import State, ObstacleType, DynamicObstacle
from commonroad.prediction.prediction import TrajectoryPrediction, Trajectory


center_vertices = np.array([[-20, 0], [-10, 0], [0, 0], [10, 0], [20, 0], [30, 0], [40, 0], [50, 0], [60, 0], [70, 0],
                            [80, 0]])
right_vertices = np.array([[-20, -2], [-10, -2], [0, -2], [10, -2], [20, -2], [30, -2], [40, -2], [50, -2], [60, -2],
                           [70, -2],
                           [80, -2]])
left_vertices = np.array([[-20, 2], [-10, 2], [0, 2], [10, 2], [20, 2], [30, 2], [40, 2], [50, 2], [60, 2], [70, 2],
                          [80, 2]])
lanelet_id = 1
lanelet_network = LaneletNetwork()
lanelet_network.add_lanelet(Lanelet(left_vertices, center_vertices, right_vertices, lanelet_id))

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

cpp_env_model.registerScenario(123, 0, lanelet_network, [other_obstacle], [ego_obstacle])

# Monitor-Mode
print(cpp_env_model.safeDistanceBooleanEvaluation(123, 0, 1, [2]))
print(cpp_env_model.safeDistanceBooleanEvaluation(123, 1, 1, [2]))
print(cpp_env_model.safeDistanceBooleanEvaluation(0, 20, 20, 20, -1, -1, 0))
print(cpp_env_model.safeDistanceBooleanEvaluation(20, 30, 20, 0, 0, 0, 0))
