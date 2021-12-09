//
// Created by Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "interstate_broad_enough_predicate.h"
#include "commonroad_cpp/predicates/position/in_front_of_predicate.h"
#include "commonroad_cpp/predicates/position/in_same_lane_predicate.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>
bool InterstateBroadEnoughPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP) {
    auto occupied_lanelets = obstacleK->getOccupiedLanelets(timeStep);
    double obsK_s = obstacleK->getStateByTimeStep(timeStep)->getLonPosition();
    for (auto lanelet : occupied_lanelets) {
        if (roadWidth(lanelet, obsK_s) <= parameters.minInterstateWidth)
            return false;
    }
    return true;
}

Constraint InterstateBroadEnoughPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                                const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("In Slow Moving Traffic Predicate does not support constraint evaluation!");
}

double InterstateBroadEnoughPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                        const std::shared_ptr<Obstacle> &obstacleK,
                                                        const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("In Slow Moving Traffic Predicate does not support robust evaluation!");
}
double InterstateBroadEnoughPredicate::roadWidth(const std::shared_ptr<Lanelet> &lanelet, double position) {
    std::vector<std::shared_ptr<Lanelet>> adj_lanelets = adjacentLanelets(lanelet);
    double road_width = 0.0;
    for (auto &l : adj_lanelets) {
        road_width += roadWidth(l, position);
    }
    return road_width;
}

std::vector<std::shared_ptr<Lanelet>>
InterstateBroadEnoughPredicate::adjacentLanelets(const std::shared_ptr<Lanelet> &lanelet) {
    std::vector<std::shared_ptr<Lanelet>> adj_lanelets;
    std::shared_ptr<Lanelet> la = lanelet;
    adj_lanelets.push_back(la);

    while (la != nullptr && la->getAdjacentLeft().adj != nullptr) {
        adj_lanelets.push_back(la);
        la = la->getAdjacentLeft().adj;
    }

    while (la != nullptr && la->getAdjacentRight().adj != nullptr) {
        adj_lanelets.push_back(la);
        la = la->getAdjacentRight().adj;
    }

    return adj_lanelets;
}

InterstateBroadEnoughPredicate::InterstateBroadEnoughPredicate(): CommonRoadPredicate(false) {}
