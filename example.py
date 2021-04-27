import os
import cpp_env_model

from commonroad.common.file_reader import CommonRoadFileReader

file_path = os.path.join(os.getcwd(), "tests/testScenarios/DEU_Muc-2_1_T-1.xml")

scenario, planning_problem_set = CommonRoadFileReader(file_path).open()

test = cpp_env_model.registerScenario(123, 0, scenario.lanelet_network, scenario.obstacles, scenario.obstacles)

print(cpp_env_model.safeDistanceBooleanEvaluation(123, 0, [scenario.obstacles[1].obstacle_id],
                                                  [scenario.obstacles[2].obstacle_id]))

