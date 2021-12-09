//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <algorithm>

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

#include "keeps_lane_speed_limit_predicate.h"

double KeepsLaneSpeedLimitPredicate::speedLimit(const std::shared_ptr<Lanelet> &lanelet,
                                                const std::string &speedLimitId) {
    double limit = parameters.maxPositiveDouble;
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = lanelet->getTrafficSigns();
    for (std::shared_ptr<TrafficSign> signPtr : trafficSigns) {
        for (std::shared_ptr<TrafficSignElement> elemPtr : signPtr->getTrafficSignElements()) {
            if (elemPtr->getId() == speedLimitId) {
                double signLimit = std::stod(elemPtr->getAdditionalValues()[0]);
                if (limit > signLimit)
                    limit = signLimit;
            }
        }
    }
    return limit;
}

double KeepsLaneSpeedLimitPredicate::speedLimit(const std::vector<std::shared_ptr<Lanelet>> &lanelets,
                                                const std::string &speedLimitId) {
    std::vector<double> speedLimits;
    for (const auto &lanelet : lanelets) {
        speedLimits.push_back(speedLimit(lanelet, speedLimitId));
    }
    return *std::min_element(speedLimits.begin(), speedLimits.end());
}

double KeepsLaneSpeedLimitPredicate::speedLimitSuggested(const std::vector<std::shared_ptr<Lanelet>> &lanelets,
                                                         const std::string &speedLimitId) {
    double vMaxLane{speedLimit(lanelets, speedLimitId)};
    if (vMaxLane == parameters.maxPositiveDouble)
        return parameters.desiredInterstateVelocity;
    else
        return std::min(parameters.desiredInterstateVelocity, vMaxLane);
}

bool KeepsLaneSpeedLimitPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP) {
    double vMaxLane{speedLimit(obstacleK->getOccupiedLanelets(world->getRoadNetwork(), timeStep),
                               world->getRoadNetwork()->extractTrafficSignIDForCountry(TrafficSignTypes::MAX_SPEED))};
    return vMaxLane >= obstacleK->getStateByTimeStep(timeStep)->getVelocity();
}

double KeepsLaneSpeedLimitPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                      const std::shared_ptr<Obstacle> &obstacleK,
                                                      const std::shared_ptr<Obstacle> &obstacleP) {
    // TODO add robust mode
    throw std::runtime_error("KeepsLaneSpeedLimitPredicate does not support robust evaluation!");
}

Constraint KeepsLaneSpeedLimitPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                              const std::shared_ptr<Obstacle> &obstacleK,
                                                              const std::shared_ptr<Obstacle> &obstacleP) {
    // TODO add constraint mode
    throw std::runtime_error("KeepsLaneSpeedLimitPredicate does not support constraint evaluation!");
}
