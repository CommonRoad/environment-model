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

#include "commonroad_cpp/roadNetwork/road_network.h"
#include "in_leftmost_lane_predicate.h"

bool InLeftmostLanePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                const std::shared_ptr<Obstacle> &obstacleP) {

    const std::shared_ptr<RoadNetwork> roadNetwork = world->getRoadNetwork();
    std::vector<std::shared_ptr<Lanelet>> lanelets = obstacleK->getOccupiedLanelets(timeStep);

    for (auto &l : lanelets) {
        std::shared_ptr<Lanelet> lanelet = roadNetwork->findLaneletById((l->getId()));
        if (lanelet->getAdjacentLeft().adj == nullptr ||
            lanelet->getAdjacentLeft().dir != lanelet->getAdjacentLeft().adj->getAdjacentRight().dir)
            return true;
    }
    return false;
}

double InLeftmostLanePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("In Leftmost Lane Predicate does not support robust evaluation!");
}

Constraint InLeftmostLanePredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                         const std::shared_ptr<Obstacle> &obstacleK,
                                                         const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("In Leftmost Lane Predicate does not support constraint evaluation!");
}

InLeftmostLanePredicate::InLeftmostLanePredicate() : CommonRoadPredicate(false) {}
