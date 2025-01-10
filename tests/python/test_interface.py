import os
import unittest
from pathlib import Path

import crcpp
import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.state import CustomState, InitialState, ExtendedPMState
from commonroad.scenario.trajectory import Trajectory


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

    def test_set_trajectory(self):
        full_path = (
            Path(__file__).parent.parent.parent
            / "tests/scenarios/predicates/DEU_TestSafeDistance-1/DEU_TestSafeDistance-1_1_T-1.pb"
        )
        scenario_path_tmp = Path(full_path)
        map_path = (
            scenario_path_tmp.parent
            / f"{scenario_path_tmp.stem.split('_')[0]}_{scenario_path_tmp.stem.split('_')[1]}.pb"
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

        world.obstacles[0].update_trajectory(
            [CustomState(time_step=4, velocity=0, position=np.array([1, 2]), orientation=0, acceleration=0)]
        )

        self.assertEqual(world.obstacles[0].get_state_by_time_step(4).x, 1)
        self.assertEqual(world.obstacles[0].get_state_by_time_step(4).y, 2)

    def test_set_current_state(self):
        full_path = (
            Path(__file__).parent.parent.parent
            / "tests/scenarios/predicates/DEU_TestSafeDistance-1/DEU_TestSafeDistance-1_1_T-1.pb"
        )
        scenario_path_tmp = Path(full_path)
        map_path = (
            scenario_path_tmp.parent
            / f"{scenario_path_tmp.stem.split('_')[0]}_{scenario_path_tmp.stem.split('_')[1]}.pb"
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

        world.obstacles[0].update_current_state(
            CustomState(time_step=4, velocity=0, position=np.array([1, 2]), orientation=0, acceleration=0)
        )

        self.assertEqual(world.obstacles[0].current_state.x, 1)
        self.assertEqual(world.obstacles[0].current_state.y, 2)

    def test_world_parameters(self):
        full_path = (
            Path(__file__).parent.parent.parent
            / "tests/scenarios/predicates/DEU_TestSafeDistance-1/DEU_TestSafeDistance-1_1_T-1.pb"
        )
        scenario_path_tmp = Path(full_path)
        map_path = (
            scenario_path_tmp.parent
            / f"{scenario_path_tmp.stem.split('_')[0]}_{scenario_path_tmp.stem.split('_')[1]}.pb"
        )
        scenario = CommonRoadFileReader(filename_dynamic=full_path, filename_map=map_path).open_map_dynamic()

        wp = crcpp.WorldParameters(
            crcpp.RoadNetworkParameters(),
            crcpp.SensorParameters(250.0, 250.0),
            crcpp.ActuatorParameters(),
            crcpp.TimeParameters(),
            crcpp.ActuatorParameters(),
        )
        world1 = crcpp.World(
            str(scenario.scenario_id), 2, 0.1, "DEU", scenario.lanelet_network, [], scenario.obstacles, wp
        )
        world2 = crcpp.World(scenario, wp)
        world3_tmp = crcpp.World(
            str(scenario.scenario_id), 2, 0.1, "DEU", scenario.lanelet_network, [], scenario.obstacles, wp
        )
        world3 = crcpp.World(str(scenario.scenario_id), 2, world3_tmp.road_network, [], world3_tmp.obstacles, 0.1, wp)

        self.assertEqual(world1.obstacles[0].sensor_parameters.fov_front, 250.0)
        self.assertEqual(world2.obstacles[0].sensor_parameters.fov_front, 250.0)
        self.assertEqual(world3.obstacles[0].sensor_parameters.fov_front, 250.0)

    def test_update_obstacles(self):
        full_path = (
            Path(__file__).parent.parent.parent
            / "tests/scenarios/predicates/DEU_TestSafeDistance-1/DEU_TestSafeDistance-1_1_T-1.pb"
        )
        scenario_path_tmp = Path(full_path)
        map_path = (
            scenario_path_tmp.parent
            / f"{scenario_path_tmp.stem.split('_')[0]}_{scenario_path_tmp.stem.split('_')[1]}.pb"
        )
        scenario = CommonRoadFileReader(filename_dynamic=full_path, filename_map=map_path).open_map_dynamic()
        obsManip = scenario.obstacles[0]
        initialTimeStepTraj = obsManip.prediction.initial_time_step
        wp = crcpp.WorldParameters(
            crcpp.RoadNetworkParameters(),
            crcpp.SensorParameters(),
            crcpp.ActuatorParameters(),
            crcpp.TimeParameters(5, 0.3),
            crcpp.ActuatorParameters(),
        )
        world1 = crcpp.World(
            str(scenario.scenario_id), 2, 0.1, "DEU", scenario.lanelet_network, [], [scenario.obstacles[0]], wp
        )

        # update obstacle and add new one
        obstacle_copy_traj = obsManip.prediction.trajectory.state_list
        obstacle_copy_state = obstacle_copy_traj[0].convert_state_to_state(InitialState())
        obstacle_copy_state.acceleration = 0
        obstacle_copy_traj.remove(obstacle_copy_traj[0])
        obstacle_copy = DynamicObstacle(
            obsManip.obstacle_id,
            obsManip.obstacle_type,
            obsManip.obstacle_shape,
            obstacle_copy_state,
            TrajectoryPrediction(
                Trajectory(obstacle_copy_traj[0].time_step, obstacle_copy_traj), obsManip.obstacle_shape
            ),
        )
        new_obstacles = [obstacle_copy, scenario.obstacles[1]]
        world1.update_obstacles(new_obstacles)
        self.assertEqual(len(world1.obstacles), 2)
        self.assertEqual(obsManip.obstacle_id, world1.obstacles[0].id)
        self.assertEqual(world1.obstacles[0].current_state.time_step, initialTimeStepTraj)
        self.assertEqual(list(world1.obstacles[0].trajectory_prediction.keys())[0], initialTimeStepTraj + 1)
        self.assertEqual(len(world1.obstacles[0].history), 1)

        # obstacle is not present anymore -> should be considered until relevant history size has passed
        for i in range(4):
            obstacle_copy_traj = obsManip.prediction.trajectory.state_list
            obstacle_copy_state = obstacle_copy_traj[0].convert_state_to_state(InitialState())
            obstacle_copy_state.acceleration = 0
            obstacle_copy_traj.remove(obstacle_copy_traj[0])
            obstacle_copy = DynamicObstacle(
                obsManip.obstacle_id,
                obsManip.obstacle_type,
                obsManip.obstacle_shape,
                obstacle_copy_state,
                TrajectoryPrediction(
                    Trajectory(obstacle_copy_traj[0].time_step, obstacle_copy_traj), obsManip.obstacle_shape
                ),
            )
            new_obstacles = [obstacle_copy]
            world1.update_obstacles(new_obstacles)
            self.assertEqual(len(world1.obstacles), 2)
        obstacle_copy_traj = obsManip.prediction.trajectory.state_list
        obstacle_copy_state = obstacle_copy_traj[0].convert_state_to_state(InitialState())
        obstacle_copy_state.acceleration = 0
        obstacle_copy_traj.remove(obstacle_copy_traj[0])
        obstacle_copy = DynamicObstacle(
            obsManip.obstacle_id,
            obsManip.obstacle_type,
            obsManip.obstacle_shape,
            obstacle_copy_state,
            TrajectoryPrediction(
                Trajectory(obstacle_copy_traj[0].time_step, obstacle_copy_traj), obsManip.obstacle_shape
            ),
        )
        new_obstacles = [obstacle_copy]
        world1.update_obstacles(new_obstacles)
        self.assertEqual(len(world1.obstacles), 1)

        # check whether new obstacle with history is used
        hist = obstacle_copy_state.convert_state_to_state(ExtendedPMState())
        obstacle_copy_traj = obsManip.prediction.trajectory.state_list
        obstacle_copy_state = obstacle_copy_traj[0].convert_state_to_state(InitialState())
        obstacle_copy_state.acceleration = 0
        obstacle_copy_traj.remove(obstacle_copy_traj[0])
        obstacle_copy = DynamicObstacle(
            obsManip.obstacle_id,
            obsManip.obstacle_type,
            obsManip.obstacle_shape,
            obstacle_copy_state,
            TrajectoryPrediction(
                Trajectory(obstacle_copy_traj[0].time_step, obstacle_copy_traj), obsManip.obstacle_shape
            ),
            history=[hist],
        )
        new_obstacles = [obstacle_copy]
        world1.update_obstacles(new_obstacles)
        self.assertEqual(len(world1.obstacles), 1)
        self.assertEqual(len(world1.obstacles[0].history), 1)
        self.assertEqual(list(world1.obstacles[0].history.values())[0].time_step, hist.time_step)
        self.assertEqual(list(world1.obstacles[0].history.keys())[0], hist.time_step)

    def test_update_obstacles_traj(self):
        full_path = (
            Path(__file__).parent.parent.parent
            / "tests/scenarios/predicates/DEU_TestSafeDistance-1/DEU_TestSafeDistance-1_1_T-1.pb"
        )
        scenario_path_tmp = Path(full_path)
        map_path = (
            scenario_path_tmp.parent
            / f"{scenario_path_tmp.stem.split('_')[0]}_{scenario_path_tmp.stem.split('_')[1]}.pb"
        )
        scenario = CommonRoadFileReader(filename_dynamic=full_path, filename_map=map_path).open_map_dynamic()
        obsManip = scenario.obstacles[0]
        initialTimeStepTraj = obsManip.prediction.initial_time_step
        wp = crcpp.WorldParameters(
            crcpp.RoadNetworkParameters(),
            crcpp.SensorParameters(),
            crcpp.ActuatorParameters(),
            crcpp.TimeParameters(5, 0.3),
            crcpp.ActuatorParameters(),
        )
        world1 = crcpp.World(
            str(scenario.scenario_id), 2, 0.1, "DEU", scenario.lanelet_network, [], [scenario.obstacles[0]], wp
        )

        # update obstacle and add new one
        obstacle_copy_traj = obsManip.prediction.trajectory.state_list
        obstacle_copy_state = obstacle_copy_traj[0]
        obstacle_copy_traj.remove(obstacle_copy_state)
        new_obstacles = [scenario.obstacles[1]]
        cstate = {obsManip.obstacle_id: obstacle_copy_state}
        traj = {obsManip.obstacle_id: obstacle_copy_traj}
        world1.update_obstacles_traj(new_obstacles, cstate, traj)
        self.assertEqual(len(world1.obstacles), 2)
        self.assertEqual(obsManip.obstacle_id, world1.obstacles[1].id)
        self.assertEqual(world1.obstacles[1].current_state.time_step, initialTimeStepTraj)
        self.assertEqual(list(world1.obstacles[1].trajectory_prediction.keys())[0], initialTimeStepTraj + 1)
        self.assertEqual(len(world1.obstacles[1].history), 1)

        # obstacle is not present anymore -> should be considered until relevant history size has passed
        new_obstacles = []
        for i in range(4):
            obstacle_copy_traj = obsManip.prediction.trajectory.state_list
            obstacle_copy_state = obstacle_copy_traj[0]
            obstacle_copy_traj.remove(obstacle_copy_state)
            cstate = {obsManip.obstacle_id: obstacle_copy_state}
            traj = {obsManip.obstacle_id: obstacle_copy_traj}
            world1.update_obstacles_traj(new_obstacles, cstate, traj)
            self.assertEqual(len(world1.obstacles), 2)
        obstacle_copy_traj = obsManip.prediction.trajectory.state_list
        obstacle_copy_state = obstacle_copy_traj[0]
        obstacle_copy_traj.remove(obstacle_copy_state)
        cstate = {obsManip.obstacle_id: obstacle_copy_state}
        traj = {obsManip.obstacle_id: obstacle_copy_traj}
        world1.update_obstacles_traj(new_obstacles, cstate, traj)
        self.assertEqual(len(world1.obstacles), 1)

    def test_world_propagate(self):
        full_path = (
            Path(__file__).parent.parent.parent
            / "tests/scenarios/predicates/DEU_TestSafeDistance-1/DEU_TestSafeDistance-1_1_T-1.pb"
        )
        scenario_path_tmp = Path(full_path)
        map_path = (
            scenario_path_tmp.parent
            / f"{scenario_path_tmp.stem.split('_')[0]}_{scenario_path_tmp.stem.split('_')[1]}.pb"
        )
        scenario = CommonRoadFileReader(filename_dynamic=full_path, filename_map=map_path).open_map_dynamic()
        wp = crcpp.WorldParameters(
            crcpp.RoadNetworkParameters(),
            crcpp.SensorParameters(),
            crcpp.ActuatorParameters(),
            crcpp.TimeParameters(5, 0.3),
            crcpp.ActuatorParameters(),
        )
        world = crcpp.World(
            str(scenario.scenario_id),
            2,
            0.1,
            "DEU",
            scenario.lanelet_network,
            [scenario.obstacles[1]],
            [scenario.obstacles[0]],
            wp,
        )
        t1obs = scenario.obstacles[0].initial_state.time_step
        t2obs = scenario.obstacles[0].state_at_time(t1obs + 1).time_step
        t1ego = scenario.obstacles[1].initial_state.time_step
        t2ego = scenario.obstacles[1].state_at_time(t1ego + 1).time_step

        world.propagate()
        self.assertEqual(len(world.obstacles[0].history), 1)
        self.assertEqual(world.obstacles[0].history[0].time_step, t1obs)
        self.assertEqual(len(world.ego_vehicles[0].history), 1)
        self.assertEqual(world.ego_vehicles[0].history[0].time_step, t1ego)
        self.assertEqual(world.obstacles[0].current_state.time_step, t2obs)
        self.assertEqual(world.ego_vehicles[0].current_state.time_step, t2ego)

        world.propagate()
        self.assertEqual(len(world.obstacles[0].history), 2)
        self.assertEqual(len(world.ego_vehicles[0].history), 2)

        world.propagate(False)
        self.assertEqual(len(world.obstacles[0].history), 3)
        self.assertEqual(len(world.ego_vehicles[0].history), 2)

        for idx in range(50):
            world.propagate()

    def test_obstacle_creation(self):
        full_path = (
            Path(__file__).parent.parent.parent
            / "tests/scenarios/predicates/DEU_TestSafeDistance-1/DEU_TestSafeDistance-1_1_T-1.pb"
        )
        scenario_path_tmp = Path(full_path)
        map_path = (
            scenario_path_tmp.parent
            / f"{scenario_path_tmp.stem.split('_')[0]}_{scenario_path_tmp.stem.split('_')[1]}.pb"
        )
        scenario = CommonRoadFileReader(filename_dynamic=full_path, filename_map=map_path).open_map_dynamic()

        obs = crcpp.Obstacle(scenario.obstacles[0])
        self.assertEqual(obs.id, scenario.obstacles[0].obstacle_id)
        self.assertEqual(obs.current_state.velocity, scenario.obstacles[0].initial_state.velocity)
        self.assertEqual(obs.get_state_by_time_step(2).velocity, scenario.obstacles[0].state_at_time(2).velocity)

    def test_shapes(self):
        rec = crcpp.Rectangle(1, 2)
        circ = crcpp.Circle(3)
        vert1 = crcpp.Vertex()
        vert1.x = 0
        vert1.y = 1
        vert2 = crcpp.Vertex()
        vert2.x = 1
        vert2.y = 0
        vert3 = crcpp.Vertex()
        vert3.x = 1
        vert3.y = 4
        poly = crcpp.Polygon([vert1, vert2, vert3])
        sgroup = crcpp.ShapeGroup([rec, circ, poly])

        self.assertEqual(len(sgroup.shapes), 3)
        self.assertEqual(circ.radius, 3)
        circ.radius = 2
        self.assertEqual(circ.radius, 2)
        self.assertEqual(rec.width, 2)
        rec.width = 3
        self.assertEqual(rec.width, 3)
        self.assertEqual(rec.length, 1)
        rec.length = 4
        self.assertEqual(rec.length, 4)
        self.assertEqual(poly.vertices[0].x, 0)
        self.assertEqual(poly.vertices[2].y, 4)
        self.assertEqual(sgroup.shapes[0].width, 3)

    def test_occupancy(self):
        rec = crcpp.Rectangle(1, 2)
        circ = crcpp.Circle(3)
        vert1 = crcpp.Vertex()
        vert1.x = 0
        vert1.y = 1
        vert2 = crcpp.Vertex()
        vert2.x = 1
        vert2.y = 0
        vert3 = crcpp.Vertex()
        vert3.x = 1
        vert3.y = 4
        poly = crcpp.Polygon([vert1, vert2, vert3])
        sgroup = crcpp.ShapeGroup([rec, circ, poly])

        occ1 = crcpp.Occupancy(1, rec)
        occ2 = crcpp.Occupancy(2, sgroup)

        self.assertEqual(occ1.time_step, 1)
        self.assertEqual(occ2.time_step, 2)

        self.assertEqual(occ1.shape.width, 2)
        self.assertEqual(len(occ2.shape.shapes), 3)

    def test_set_based_prediction(self):
        file_path1 = Path(__file__).parent.parent.parent / "tests/scenarios/USA_Lanker-1_1_S-1.xml"
        scenario1, _ = CommonRoadFileReader(filename_2020a=file_path1).open()
        world1 = crcpp.World(
            str(scenario1.scenario_id),
            2,
            0.1,
            "DEU",
            scenario1.lanelet_network,
            [],
            scenario1.obstacles,
        )

        for obs in world1.obstacles:
            sp = obs.set_based_prediction
            self.assertEqual(len(sp), len(scenario1.obstacle_by_id(obs.id).prediction.occupancy_set))
            idx = 0
            for time_step, occ in sp.items():
                self.assertEqual(time_step, scenario1.obstacle_by_id(obs.id).prediction.occupancy_set[idx].time_step)
                idx += 1

        sp = world1.obstacles[0].set_based_prediction
        idx = 0
        self.assertEqual(
            len(list(sp.values())[0].shape.vertices),
            len(scenario1.obstacle_by_id(world1.obstacles[0].id).prediction.occupancy_set[0].shape.vertices),
        )
        for vert in list(sp.values())[0].shape.vertices:
            self.assertEqual(
                vert.x,
                scenario1.obstacle_by_id(world1.obstacles[0].id).prediction.occupancy_set[0].shape.vertices[idx][0],
            )
            self.assertEqual(
                vert.y,
                scenario1.obstacle_by_id(world1.obstacles[0].id).prediction.occupancy_set[0].shape.vertices[idx][1],
            )
            idx += 1

        self.assertEqual(
            len(list(sp.values())[1].shape.shapes),
            len(scenario1.obstacle_by_id(world1.obstacles[0].id).prediction.occupancy_set[1].shape.shapes),
        )


if __name__ == "__main__":
    unittest.main()
