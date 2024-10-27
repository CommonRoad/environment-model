from pathlib import Path

import crcpp
from commonroad.common.file_reader import CommonRoadFileReader
from crcpp import IsOfTypePredicate

full_path = "/media/sebastian/TUM/06_code/internal/environment-model/tests/scenarios/ZAM_TestReadingAll-1/ZAM_TestReadingAll-1_1_T-1.pb"
scenario_path_tmp = Path(full_path)
map_path = (
    scenario_path_tmp.parent / f"{scenario_path_tmp.stem.split('_')[0]}_{scenario_path_tmp.stem.split('_')[1]}.pb"
)
scenario = CommonRoadFileReader(filename_dynamic=full_path, filename_map=map_path).open_map_dynamic()
print(scenario.scenario_id)
world1 = crcpp.World(
    str(scenario.scenario_id),
    2,
    0.1,
    "DEU",
    scenario.lanelet_network,
    [],
    scenario.obstacles,
)

obstacle = world1.obstacles[0]
pred = IsOfTypePredicate.boolean_evaluation(0, world1, obstacle, obstacle, ["car"])
