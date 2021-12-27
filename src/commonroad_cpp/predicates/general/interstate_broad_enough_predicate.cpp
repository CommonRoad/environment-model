//
// Created by Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "interstate_broad_enough_predicate.h"
#include "commonroad_cpp/predicates/position/in_front_of_predicate.h"
#include "commonroad_cpp/predicates/position/in_same_lane_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

bool InterstateBroadEnoughPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP) {
    auto occupied_lanelets = obstacleK->getOccupiedLanelets(timeStep);
    double obsK_s = obstacleK->getStateByTimeStep(timeStep)->getLonPosition();
    return std::all_of(occupied_lanelets.begin(), occupied_lanelets.end(),
                       [obsK_s, world, this](const std::shared_ptr<Lanelet> &lanelet) {
                           return roadWidth(world, lanelet, obsK_s) > parameters.minInterstateWidth;
                       });
}

Constraint InterstateBroadEnoughPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                                const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("Interstate Broad Enough Predicate does not support constraint evaluation!");
}

double InterstateBroadEnoughPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                        const std::shared_ptr<Obstacle> &obstacleK,
                                                        const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("Interstate Broad Enough Traffic Predicate does not support robust evaluation!");
}

double InterstateBroadEnoughPredicate::roadWidth(const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Lanelet> &lanelet, double position) {
    const std::shared_ptr<RoadNetwork> roadNetwork = world->getRoadNetwork();
    std::vector<std::shared_ptr<Lanelet>> adj_lanelets = adjacentLanelets(lanelet);
    double road_width = lanelet->getWidth(position);
    for (auto &adjLanelet : adj_lanelets)
        road_width += roadNetwork->findLanesByContainedLanelet(adjLanelet->getId())[0]->getWidth(position);

    return road_width;
}

std::vector<std::shared_ptr<Lanelet>>
InterstateBroadEnoughPredicate::adjacentLanelets(const std::shared_ptr<Lanelet> &lanelet) {
    std::vector<std::shared_ptr<Lanelet>> adj_lanelets;
    std::shared_ptr<Lanelet> laneletTmp = lanelet;

    while (laneletTmp != nullptr && laneletTmp->getAdjacentLeft().adj != nullptr) {
        adj_lanelets.push_back(laneletTmp);
        laneletTmp = laneletTmp->getAdjacentLeft().adj;
    }

    laneletTmp = lanelet;

    while (laneletTmp != nullptr && laneletTmp->getAdjacentRight().adj != nullptr) {
        adj_lanelets.push_back(laneletTmp);
        laneletTmp = laneletTmp->getAdjacentRight().adj;
    }
    return adj_lanelets;
}

InterstateBroadEnoughPredicate::InterstateBroadEnoughPredicate() : CommonRoadPredicate(false) {}