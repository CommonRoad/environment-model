import unittest
import os
from pathlib import Path

import crcpp
from commonroad.common.file_reader import CommonRoadFileReader


class TestPythonInterface(unittest.TestCase):
    def setUp(self):
        self.path = os.path.dirname(os.path.realpath(__file__)) + '/../scenarios/'
        self.filenames = ['DEU_Muc-2/DEU_Muc-2_1_T-1.pb', 'ZAM_Urban-2/ZAM_Urban-2_1_T-1.pb',
                          'ZAM_TestReadingAll-1/ZAM_TestReadingAll-1_1_T-1.pb',
                          'ZAM_Tjunction-1/ZAM_Tjunction-1_47_T-1.pb']

    def test_scenario_registration(self):
        scenario_id = 123
        for scenario in self.filenames:
            full_path = self.path + scenario
            scenario_path_tmp = Path(full_path)
            map_path = (
                    scenario_path_tmp.parent
                    / f"{scenario_path_tmp.stem.split('_')[0]}_{scenario_path_tmp.stem.split('_')[1]}.pb"
            )
            scenario = CommonRoadFileReader(filename_dynamic=full_path, filename_map=map_path).open_map_dynamic()
            try:
                print("Converting - " + full_path)
                crcpp.register_scenario(scenario_id, str(scenario.scenario_id), 0, scenario.dt, "DEU", scenario.lanelet_network,
                                        scenario.obstacles, [])
                crcpp.remove_scenario(123)
                print("Successful")
            except:
                print("Failed")


if __name__ == '__main__':
    unittest.main()
