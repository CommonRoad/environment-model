import os
import cpp_env_model

from commonroad.common.file_reader import CommonRoadFileReader

file_path = os.path.join(os.getcwd(), "tests/testScenarios/DEU_base_intersection.xml")

scenario, planning_problem_set = CommonRoadFileReader(file_path).open()

test = cpp_env_model.registerScenario(123, scenario.lanelet_network, scenario.dynamic_obstacles)

print(test)
