# commonroad-collision-checker (install from https://gitlab.lrz.de/tum-cps/commonroad-collision-checker)
from commonroad_cc.collision_detection.pycrcc_collision_dispatch import create_collision_checker
from commonroad_cc.visualization.draw_dispatch import draw_object as draw_object_cc

from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.visualization.draw_dispatch_cr import draw_object

import matplotlib.pyplot as plt
import numpy as np


def compute_field_of_view(cr_scenario: Scenario, cr_planning_problem_set: PlanningProblem, sensor_range: float = 50,
                          num_measurement_points: int = 360):
    # create sensor_area as circle with radius of sensor_range and discretization with num_measurement_points
    central_angle = 2 * np.pi / num_measurement_points
    sensor_area = np.empty([num_measurement_points + 1, 2], float)
    for i in range(num_measurement_points + 1):  # clock-wise oder of vertices
        sensor_area[i][0] = sensor_range * np.sin(i * central_angle + cr_planning_problem_set.initial_state.orientation) \
                            + cr_planning_problem_set.initial_state.position[0]
        sensor_area[i][1] = sensor_range * np.cos(i * central_angle + cr_planning_problem_set.initial_state.orientation) \
                            + cr_planning_problem_set.initial_state.position[1]
    # x, y = sensor_area.T  # only for plotting
    # plt.plot(x, y, 'b')

    # create collision_checker from obstacles in cr_scenario
    collision_checker = create_collision_checker(cr_scenario)
    draw_object_cc(collision_checker.time_slice(0))
    draw_object(cr_planning_problem_set)

    # compute area that can be observed from the ego vehicle, i.e. where the sensor points are not blocked by obstacles
    field_of_view = np.empty([sensor_area.shape[0], 2], float)
    for i in range(sensor_area.shape[0]):
        # colliding_segment is the part of the line ego_position - sensor_area[i] that collides
        colliding_segment = collision_checker.time_slice(0).raytrace(cr_planning_problem_set.initial_state.position[0],
                                                                     cr_planning_problem_set.initial_state.position[1],
                                                                     sensor_area[i][0], sensor_area[i][1], True)
        if colliding_segment:
            # select vertice that is behind the obstacle (i.e. further away from ego initial position)
            if np.linalg.norm([colliding_segment[0][0] - cr_planning_problem_set.initial_state.position[0],
                               colliding_segment[0][1] - cr_planning_problem_set.initial_state.position[1]]) <= \
                    np.linalg.norm([colliding_segment[0][2] - cr_planning_problem_set.initial_state.position[0],
                                    colliding_segment[0][3] - cr_planning_problem_set.initial_state.position[1]]):
                field_of_view[i][0] = colliding_segment[0][2]
                field_of_view[i][1] = colliding_segment[0][3]
            else:
                field_of_view[i][0] = colliding_segment[0][0]
                field_of_view[i][1] = colliding_segment[0][1]
            # plt.plot(field_of_view[i][0], field_of_view[i][1], 'rx')  # only for plotting
            # ToDo: for i-1 on first and i+1 on last colliding_segment, rotate colliding_segment[2] and [3] by
            #  central_angle to over-approximate the obstacles
            # optionally, on this i-1 and i+1, also append field_of_view with sensor_area[i] (to have straight edges)
        else:
            field_of_view[i][0] = sensor_area[i][0]
            field_of_view[i][1] = sensor_area[i][1]
    # x, y = field_of_view.T  # only for plotting
    # plt.plot(x, y, 'r')
    # plt.autoscale()
    # plt.axis('equal')
    # plt.show()

    return field_of_view
