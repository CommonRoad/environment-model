import unittest
import os

import cpp_env_model
from commonroad.common.file_reader import CommonRoadFileReader


class TestAtRedLeftTrafficLightPredicates(unittest.TestCase):
    def setUp(self):
        self.path = os.path.dirname(os.path.realpath(__file__)) + '/../scenarios/'
        self.filenames = ['DEU_TrafficLightTest-1_1_T-1.xml', 'ZAM_Urban-2_1.xml', 'ZAM_Tjunction-1_47_T-1.xml']

    def test_scenario_registration(self):
        scenario_id = 123
        for scenario in self.filenames:
            full_path = self.path + scenario
            sc, _ = CommonRoadFileReader(full_path).open()
            try:
                print("Converting - " + full_path)
                cpp_env_model.register_scenario(scenario_id, 0, "DEU", sc.lanelet_network, sc.obstacles, [])
                cpp_env_model.remove_scenario(123)
                print("Successful")
            except:
                print("Failed")
