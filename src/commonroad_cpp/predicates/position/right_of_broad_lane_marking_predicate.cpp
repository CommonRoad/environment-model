//
// Created by Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>
#include <commonroad_cpp/world.h>

#include "right_of_broad_lane_marking_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"

bool RightOfBroadLaneMarkingPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                            const std::shared_ptr<Obstacle> &obstacleK,const std::shared_ptr<Obstacle> &obstacleP) {

    const std::shared_ptr<RoadNetwork> roadNetwork = world->getRoadNetwork();
    std::vector<std::shared_ptr<Lanelet>> lanelets_occ = obstacleK->getOccupiedLanelets(timeStep);

    for (auto &l : lanelets_occ){
        auto lanelet = roadNetwork->findLaneletById(l->getId());
        const LineMarking lanelet_right_marking = lanelet->getLineMarkingRight();
        if (lanelet_right_marking == LineMarking::broad_dashed or lanelet_right_marking == LineMarking::solid)
            return false;
    }

    std::vector<std::shared_ptr<Lanelet>> lanelets_left_of_veh = laneletsLeftOfVehicle(timeStep, world, obstacleK);
    for (auto &lanelet : lanelets_left_of_veh) {
        const LineMarking lanelet_right_marking = lanelet->getLineMarkingRight();
        if (lanelet_right_marking==LineMarking::broad_dashed or lanelet_right_marking==LineMarking::broad_solid)
            return true;
    }
    return false;
}

double RightOfBroadLaneMarkingPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                             const std::shared_ptr<Obstacle> &obstacleK,
                                             const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("Right of broad lane marking does not support robust evaluation!");
}

Constraint RightOfBroadLaneMarkingPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("Right of broad lane marking does not support constraint evaluation!");
}


std::vector<std::shared_ptr<Lanelet>> RightOfBroadLaneMarkingPredicate::laneletsLeftOfVehicle(size_t timeStep, const std::shared_ptr<World> &world,
                                                        const std::shared_ptr<Obstacle> &obs) {
    std::vector<std::shared_ptr<Lanelet>> leftLanelets;
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets = obs->getOccupiedLanelets(timeStep);
    const std::shared_ptr<RoadNetwork> roadNetwork = world->getRoadNetwork();

    for (auto &occ_l : occupiedLanelets){
        std::vector<std::shared_ptr<Lanelet>> newLanelets = laneletsLeftOfLanet(world, occ_l);
        for (auto &nl : newLanelets){
            leftLanelets.push_back(nl);
        }
    }
    return leftLanelets;
}

std::vector<std::shared_ptr<Lanelet>> RightOfBroadLaneMarkingPredicate::laneletsLeftOfLanet(const std::shared_ptr<World> &world,
                                                      const std::shared_ptr<Lanelet> &lanelet) {
    const std::shared_ptr<RoadNetwork> roadNetwork = world->getRoadNetwork();
    std::vector<std::shared_ptr<Lanelet>> leftLanelets;
    std::shared_ptr<Lanelet> tmp_lanelet = lanelet;
    while (tmp_lanelet->getAdjacentLeft().adj!= nullptr){
        tmp_lanelet = roadNetwork->findLaneletById(tmp_lanelet->getId());
        leftLanelets.push_back(tmp_lanelet);
    }
    return leftLanelets;
}
