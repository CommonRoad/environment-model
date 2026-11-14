import argparse
from pathlib import Path

import crcpp
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer
from matplotlib import pyplot as plt


def plot_ref(obs, time_step, sc, output_dir):
    rnd = MPRenderer(figsize=(25, 10))
    rnd.draw_params.time_begin = time_step
    sc.lanelet_network.draw(rnd)
    sc.obstacle_by_id(obs.id).draw(rnd)
    rnd.render(show=False)
    x = [vert.x for vert in obs.reference_lane_by_time_step(world.road_network, time_step).center_vertices]
    y = [vert.y for vert in obs.reference_lane_by_time_step(world.road_network, time_step).center_vertices]
    plt.plot(x, y, zorder=1000)

    plt.savefig(output_dir / f"ref{obs.id}_{time_step}.png")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot the reference lanes of all obstacles in a scenario.")
    parser.add_argument("scenario", type=Path, help="path to the CommonRoad scenario file")
    parser.add_argument("--output-dir", type=Path, default=Path("."), help="directory for the generated images")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    scenario_path_tmp = args.scenario
    map_path = (
        scenario_path_tmp.parent / f"{scenario_path_tmp.stem.split('_')[0]}_{scenario_path_tmp.stem.split('_')[1]}.pb"
    )
    scenario, _ = CommonRoadFileReader(filename_2020a=scenario_path_tmp).open()

    world = crcpp.World(
        str(scenario.scenario_id),
        2,
        0.1,
        "DEU",
        scenario.lanelet_network,
        [],
        scenario.obstacles,
    )

    for rel_obs in world.obstacles:
        time_step_start = rel_obs.get_time_steps()[0]
        time_step_end = rel_obs.get_time_steps()[-1]
        plot_ref(rel_obs, time_step_start, scenario, args.output_dir)
        plot_ref(rel_obs, time_step_end, scenario, args.output_dir)
