from pathlib import Path

import crcpp
from commonroad.common.file_reader import CommonRoadFileReader

full_path = Path(__file__).parent / "tests/scenarios/predicates/DEU_TestSafeDistance-1/DEU_TestSafeDistance-1_1_T-1.pb"
scenario_path_tmp = Path(full_path)
map_path = (
    scenario_path_tmp.parent / f"{scenario_path_tmp.stem.split('_')[0]}_{scenario_path_tmp.stem.split('_')[1]}.pb"
)
scenario = CommonRoadFileReader(filename_dynamic=full_path, filename_map=map_path).open_map_dynamic()

world = crcpp.World(
    str(scenario.scenario_id),
    2,
    0.1,
    "DEU",
    scenario.lanelet_network,
    [],
    scenario.obstacles,
)

print(crcpp.InSameLanePredicate().boolean_evaluation(0, world, world.obstacles[0], world.obstacles[1]))
print(crcpp.InSameLanePredicate().boolean_evaluation(0, world, world.obstacles[0], world.obstacles[5]))
