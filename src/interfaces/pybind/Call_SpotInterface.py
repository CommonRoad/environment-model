# ------------------------------- User settings ----------------------------------- #
scenario_name = 'DEU_Muc-3_1_T-1'
scenario_path = './unittest_data/'  # or use path/to/commonroad_scenarios

start_time = 0.0
time_step_size = 0.1
end_time = 15 * time_step_size + start_time
num_threads = 4

log_and_print_debbuging_information = False  # whether you want SPOT to print the debug logs
update_obstacle_parameters = False  # whether you want to modify the default predition prameters
consider_occlusion = False  # whether you want SPOT to create phantom obstacles
print_predicted_velocity_intervals = False  # whether you want SPOT to output the velocity intervals in the terminal
write_output_to_xml = False  # whether you want SPOT to create an xml file in ./results

# ------------------------------- Import statements ----------------------------------- #
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.trajectory import State
from commonroad.geometry.shape import Rectangle, Polygon, ShapeGroup
from commonroad.prediction.prediction import Occupancy, SetBasedPrediction
from commonroad.visualization.draw_dispatch_cr import draw_object

if consider_occlusion:
    from interface.python.Helper_SpotInterface import compute_field_of_view  # requires commonroad-collision-checker

import spot

import numpy as np
import matplotlib.pyplot as plt

# ------------------------------- Open commonraod (cr) scenario ----------------------------------- #
cr_scenario, cr_planning_problem_set = CommonRoadFileReader(scenario_path + scenario_name + '.xml').open()
cr_planning_problem_set = list(cr_planning_problem_set.planning_problem_dict.values())[0]

# --------------------------- (Optionally) create field-of-view of the ego vehicle ----------------------------------- #
field_of_view = np.empty([0, 2], float)
if consider_occlusion:
    # sensor_area is (currently) circle with radius of sensor_range and discretization with num_measurement_points
    field_of_view = compute_field_of_view(cr_scenario, cr_planning_problem_set, sensor_range=75,
                                          num_measurement_points=360)

# --------------------------- Call SPOT-Interface ----------------------------------- #
if log_and_print_debbuging_information:
    spot.setLoggingMode(1)  # default is false (unless compile configuration is DEBUG)

spot_scenario_id = 1
test = spot.registerScenario(spot_scenario_id, cr_scenario.lanelet_network, cr_scenario.dynamic_obstacles,
                             [cr_planning_problem_set], field_of_view)  # Todo: use keywords like ScenarioID= ...
print('Register Scenario finish with status:', test)
if test:
    quit()

# --------------------------- Update obstacle params ----------------------------------- #
if update_obstacle_parameters:
    # Play around a bit with these params and see what happens.. (see Scenario::updateProperties() for details)
    update_dict = {
        "vehicle": {
            0: {  # 0 means that all vehicles will be changed
                "a_max": 6.0,
                "compute_occ_m1": True,
                "compute_occ_m2": True,
                "compute_occ_m3": True
            }
        },
        "egoVehicle": {
            0: {  # ID is ignored for ego vehicle (which is created based on cr_planning problem)
                "a_max": 1.0,
                "length": 5.0,
                "width": 2.0
            }
        }
    }
    test = spot.updateProperties(spot_scenario_id, update_dict)
    print('Update Properties finished with status:', test)

# --------------------------- Occupancy calculation ----------------------------------- #
print('Calling SPOT prediction with ', num_threads, 'threads and for time interval [', start_time, ', ', end_time,
      '] with step size of ', time_step_size)
# py::list py_doOccupancyPrediction(py::int_ &ScenarioID, py::float_ &StartTime, py::float_ &TimeStep, py::float_ &EndTime, py::int_ &NumThreads)
test = spot.doOccupancyPrediction(spot_scenario_id, float(start_time), float(time_step_size), float(end_time),
                                  num_threads)

# copy prediction into dynamic obstacles of commonroad scenario
print('Number of predicted obstacles:', len(test))
k = 0  # iterator over cr_dynamic_obstacles
phantom_obstacles_id_begin = 50000000  # to avoid non-unique ids in cr_scenario
for cpp_obstacle in test:
    # print('Copying output for obstacle with ID ', cpp_obstacle[0], 'to CommonRoad scenario')

    # initialise
    occupancy_list = []
    # print(int(end_time/time_step_size))
    for i in range(int(end_time / time_step_size) + 1):
        # ToDo: check if is correct that len(cpp_obstacle[1]) == (end_time / scenario.dt) + 1
        occ = Occupancy(i + 1, ShapeGroup([]))
        occupancy_list.append(occ)
    # print(len(occupancy_list))

    # all occupancy polygons (cpp_obstacle[1]) are stored in one list; thus, we need to separate each polygon
    i = 0  # iterator over occupancy_list
    for vertices_at_time_step in cpp_obstacle[1]:
        # print('Occupancy for timestep :',i)
        # print('Size of vertices: ', len(vertices_at_time_step[1]))
        # print(vertices_at_time_step[1])
        j = 1  # iterator over vertices_at_time_step
        b = 0  # index to select vertices_at_time_step that are the start of a new polygon
        while j < len(vertices_at_time_step[1]):
            compare_vertice = vertices_at_time_step[1][b]  # first vertice of next polygon
            if compare_vertice[0] == vertices_at_time_step[1][j][0] and compare_vertice[1] == \
                    vertices_at_time_step[1][j][1]:
                if (j + 1) - b < 3:  # polygon has less than 3 vertices
                    print('Warning: one duplicated vertice skipped when copying predicted occupancies to CommonRoad')
                    b += 1  # try next vertice as first vertice (in case of equal vertices directly after each other)
                else:
                    shape_obj = Polygon(np.array(vertices_at_time_step[1][b:j + 1]))
                    occupancy_list[i].shape.shapes.append(shape_obj)
                    j += 1
                    b = j
            j += 1
        assert b == j - 1, ('Last polygon not closed (at time_step = ', i, ', b = ', b)
        i += 1

        # extract predicted velocity interval (not yet supported by commonroad)
        if print_predicted_velocity_intervals:
            lane_idx = 0  # velocity is yet the same for all lanes
            v_min = vertices_at_time_step[0][lane_idx][1]
            v_max = vertices_at_time_step[0][lane_idx][2]
            print("predicted velocity interval of obstacle with id =", cpp_obstacle[0], "at time_step =", i, ": [",
                  v_min, ",", v_max, "]")

    # for phantom object from spot, create new cr_obstacle
    if k >= len(cr_scenario.dynamic_obstacles):
        cr_scenario.add_objects(DynamicObstacle(k + 1 + phantom_obstacles_id_begin, ObstacleType.UNKNOWN,
                                                Rectangle(0, 0), State(position=np.array([0, 0]), orientation=0,
                                                                       time_step=0)))

    # set occupancy as prediction of cr_obstacle
    cr_scenario.dynamic_obstacles[k].prediction = SetBasedPrediction(spot_scenario_id, occupancy_list[0:])
    k += 1

# --------------------------- write and display result ----------------------------------- #
if write_output_to_xml:
    test = spot.writeScenarioToXML(spot_scenario_id, scenario_name)
    print('write Spot Scenario to XML finished with status:', test)

plt.clf()
plt.cla()
plt.close()
fig, ax = plt.subplots(1)
draw_object(cr_scenario)
draw_object(cr_planning_problem_set)
if consider_occlusion:
    x, y = field_of_view.T
    plt.plot(x, y, 'r')
ax.autoscale_view()
# plt.axis('off')
plt.gca().set_aspect('equal')
plt.show()

# --------------------------- remove cpp scenario ----------------------------------- #
test = spot.removeScenario(spot_scenario_id)
print('Remove Spot Scenario finished with status:', test)
