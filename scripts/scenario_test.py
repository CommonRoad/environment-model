import cpp_env_model
import os
from commonroad.common.file_reader import CommonRoadFileReader
from joblib import Parallel, delayed

scenarios_2020a = "/media/sebastian/TUM/06_code/cps/scenarios/cr-scenarios/scenarios"
scenarios_2018b = "TODO"

factory_2020a = scenarios_2020a + "/scenario-factory"
hand_crafted_2020a = scenarios_2020a + "/hand-crafted"
ngsim_lankershim_2020a = scenarios_2020a + "/NGSIM/Lankershim"
ngsim_us101_2020a = scenarios_2020a + "/NGSIM/US101"
ngsim_peachtree_2020a = scenarios_2020a + "/NGSIM/Peachtree"
bicycle_2020a = scenarios_2020a + "/THI-Bicycle"

cooperative_2018b = scenarios_2018b + "/cooperative"
bicycle_2018b = scenarios_2018b + "/THI-Bicycle"
sumo_2018b = scenarios_2018b + "/SUMO"
hand_crafted_2018b = scenarios_2018b + "/hand-crafted"
ngsim_lankershim_2018b = scenarios_2018b + "/NGSIM/Lankershim"
ngsim_us101_2018b = scenarios_2018b + "/NGSIM/US101"
ngsim_peachtree_2018b = scenarios_2018b + "/NGSIM/Peachtree"

scenario_id = 123
scenario_list = []


def eval_scenario(scenario_path: str):
    try:
        sc, _ = CommonRoadFileReader(full_path).open()
        cpp_env_model.register_scenario(scenario_id, 0, "DEU", sc.lanelet_network, sc.obstacles, [])
        cpp_env_model.remove_scenario(scenario_id)
        print("Successful - " + scenario_path)
    except:
        print("Failed - " + scenario_path)


# collect scenarios
for scenario in os.listdir(hand_crafted_2020a):
    if "_S-" in scenario:
        continue
    full_path = hand_crafted_2020a + "/" + scenario
    scenario_list.append(full_path)

for scenario in os.listdir(ngsim_lankershim_2020a):
    if "_S-" in scenario:
        continue
    full_path = ngsim_lankershim_2020a + "/" + scenario
    scenario_list.append(full_path)

for scenario in os.listdir(ngsim_us101_2020a):
    if "_S-" in scenario:
        continue
    full_path = ngsim_us101_2020a + "/" + scenario
    scenario_list.append(full_path)

for scenario in os.listdir(ngsim_peachtree_2020a):
    if "_S-" in scenario:
        continue
    full_path = ngsim_peachtree_2020a + "/" + scenario
    scenario_list.append(full_path)

for scenario in os.listdir(bicycle_2020a):
    if "_S-" in scenario:
        continue
    full_path = bicycle_2020a + "/" + scenario
    scenario_list.append(full_path)

for scenario in os.listdir(factory_2020a):
    if "_S-" in scenario:
        continue
    full_path = factory_2020a + "/" + scenario
    scenario_list.append(full_path)

Parallel(n_jobs=6)(delayed(eval_scenario)(path) for path in scenario_list)

# for scenario in os.listdir(cooperative_2018b):
#     full_path = cooperative_2018b + "/" + scenario
#     CommonRoadFileReader(full_path).open()
#
# for scenario in os.listdir(sumo_2018b):
#     full_path = sumo_2018b + "/" + scenario
#     CommonRoadFileReader(full_path).open()
#
# for scenario in os.listdir(bicycle_2018b):
#     full_path = bicycle_2018b + "/" + scenario
#     CommonRoadFileReader(full_path).open()
#
# for scenario in os.listdir(ngsim_lankershim_2018b):
#     full_path = ngsim_lankershim_2018b + "/" + scenario
#     CommonRoadFileReader(full_path).open()
#
# for scenario in os.listdir(ngsim_us101_2018b):
#     full_path = ngsim_us101_2018b + "/" + scenario
#     CommonRoadFileReader(full_path).open()
#
# for scenario in os.listdir(ngsim_peachtree_2018b):
#     full_path = ngsim_peachtree_2018b + "/" + scenario
#     CommonRoadFileReader(full_path).open()
#
# for scenario in os.listdir(hand_crafted_2018b):
#     full_path = hand_crafted_2018b + "/" + scenario
#     CommonRoadFileReader(full_path).open()
#
