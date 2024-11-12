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

    def test_predicates(self):
        for scenario in self.filenames:
            full_path = Path(self.path + scenario)
            (obstacles, road_network, tss) = crcpp.read_scenario(str(full_path))
            world = crcpp.World("foo", 0, road_network, [], obstacles, tss)

            predicate = crcpp.OnSameRoadPredicate()
            for obs_a, obs_b in [(a, b) for a in world.obstacles for b in world.obstacles]:
                if obs_a.type != crcpp.ObstacleType.car or obs_b.type != crcpp.ObstacleType.car:
                    continue

                obs_a_ts = frozenset(obs_a.get_time_steps())
                obs_b_ts = frozenset(obs_b.get_time_steps())
                valid_ts = frozenset.intersection(obs_a_ts, obs_b_ts)

                for ts in valid_ts:
                    result = predicate.boolean_evaluation(ts, world, obs_a, obs_b)  # noqa: F841

    def test_scenario_binding(self):
        for scenario in self.filenames:
            full_path = self.path + scenario
            scenario_path_tmp = Path(full_path)
            map_path = (
                scenario_path_tmp.parent
                / f"{scenario_path_tmp.stem.split('_')[0]}_{scenario_path_tmp.stem.split('_')[1]}.pb"
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
            self.assertEqual(world1.time_step, 2)

            world2 = crcpp.World(scenario)
            self.assertEqual(len(world1.road_network.lanelets), len(world2.road_network.lanelets))
            self.assertEqual(len(world1.obstacles), len(world2.obstacles))

    def test_create_world(self):
        file_path = Path(
            os.path.dirname(os.path.realpath(__file__))
            + "/../scenarios/DEU_TwoLanesWithDifferentOrientation-1_1_T-1.xml"
        )
        world1 = crcpp.create_world(file_path.__str__())
        self.assertGreater(len(world1.road_network.lanelets), 1)

        file_path = Path(os.path.dirname(os.path.realpath(__file__)) + "/../scenarios/BEL_Zwevegem-1_5_T-1.xml")
        world2 = crcpp.create_world(file_path.__str__())
        self.assertGreater(len(world2.road_network.lanelets), 2)
        self.assertGreater(len(world2.road_network.traffic_signs), 1)

    def test_all(self):
        paths = Path("/media/sebastian/TUM/06_code/scenarios/highD/scenarios").rglob("*.xml")
        print("load xml directly")
        for file_path in paths:
            crcpp.create_world(file_path.__str__())
        print("python binding")
        for file_path in paths:
            scenario, _ = CommonRoadFileReader(filename_2020a=file_path).open()
            crcpp.World(
                str(scenario.scenario_id),
                2,
                0.1,
                "DEU",
                scenario.lanelet_network,
                [],
                scenario.obstacles,
            )


if __name__ == "__main__":
    unittest.main()
