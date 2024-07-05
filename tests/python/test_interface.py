import os
import unittest
from pathlib import Path

import crcpp
from commonroad.common.file_reader import CommonRoadFileReader


class TestPythonInterface(unittest.TestCase):
    def setUp(self):
        self.path = os.path.dirname(os.path.realpath(__file__)) + "/../scenarios/"
        self.filenames = [
            "DEU_Muc-2/DEU_Muc-2_1_T-1.pb",
            "ZAM_Urban-2/ZAM_Urban-2_1_T-1.pb",
            "ZAM_TestReadingAll-1/ZAM_TestReadingAll-1_1_T-1.pb",
            "ZAM_Tjunction-1/ZAM_Tjunction-1_47_T-1.pb",
        ]

    def test_scenario_binding(self):
        for scenario in self.filenames:
            full_path = self.path + scenario
            scenario_path_tmp = Path(full_path)
            map_path = (
                scenario_path_tmp.parent
                / f"{scenario_path_tmp.stem.split('_')[0]}_{scenario_path_tmp.stem.split('_')[1]}.pb"
            )
            scenario = CommonRoadFileReader(
                filename_dynamic=full_path, filename_map=map_path
            ).open_map_dynamic()
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
            self.assertEquals(world1.time_step, 2)

            world2 = crcpp.World(scenario)
            self.assertEqual(
                len(world1.road_network.lanelets), len(world2.road_network.lanelets)
            )
            self.assertEqual(len(world1.obstacles), len(world2.obstacles))

    def test_create_world(self):
        file_path = Path(
            os.path.dirname(os.path.realpath(__file__))
            + "/../scenarios/DEU_TwoLanesWithDifferentOrientation-1_1_T-1.xml"
        )
        world1 = crcpp.create_world(file_path.__str__())
        self.assertGreater(len(world1.road_network.lanelets), 1)

        file_path = Path(
            os.path.dirname(os.path.realpath(__file__))
            + "/../scenarios/BEL_Zwevegem-1_5_T-1.xml"
        )
        world2 = crcpp.create_world(file_path.__str__())
        self.assertGreater(len(world2.road_network.lanelets), 2)
        self.assertGreater(len(world2.road_network.traffic_signs), 1)


if __name__ == "__main__":
    unittest.main()
