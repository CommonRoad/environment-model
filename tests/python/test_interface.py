import unittest
import os

import crcpp
from commonroad.common.file_reader import CommonRoadFileReader


class TestPythonInterface(unittest.TestCase):
    def setUp(self):
        self.path = os.path.dirname(os.path.realpath(__file__)) + '/../scenarios/'
        self.filenames = ['DEU_Muc-2_1_T-1.xml', 'ZAM_Urban-2_1.xml']

    def test_scenario_registration(self):
        scenario_id = 123
        for scenario in self.filenames:
            full_path = self.path + scenario
            sc, _ = CommonRoadFileReader(full_path).open()
            try:
                print("Converting - " + full_path)
                crcpp.register_scenario(scenario_id, 0, sc.dt, "DEU", sc.lanelet_network, sc.obstacles, [])
                crcpp.remove_scenario(123)
                print("Successful")
            except:
                print("Failed")
