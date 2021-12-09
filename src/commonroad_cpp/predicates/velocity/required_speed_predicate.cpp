//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <algorithm>

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

#include "required_speed_predicate.h"

double RequiredSpeedPredicate::requiredVelocity(const std::shared_ptr<Lanelet> &lanelet,
                                                const std::string &speedLimitId) {
    double limit = 0;
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = lanelet->getTrafficSigns();
    for (std::shared_ptr<TrafficSign> signPtr : trafficSigns) {
        for (std::shared_ptr<TrafficSignElement> elemPtr : signPtr->getTrafficSignElements()) {
            if (elemPtr->getId() == speedLimitId) {
                double signLimit = std::stod(elemPtr->getAdditionalValues()[0]);
                if (limit < signLimit)
                    limit = signLimit;
            }
        }
    }
    return limit;
}

double RequiredSpeedPredicate::requiredVelocity(const std::vector<std::shared_ptr<Lanelet>> &lanelets,
                                                const std::string &speedLimitId) {
    std::vector<double> speedLimits;
    for (const auto &lanelet : lanelets) {
        speedLimits.push_back(requiredVelocity(lanelet, speedLimitId));
    }
    return *std::min_element(speedLimits.begin(), speedLimits.end());
}

bool RequiredSpeedPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                               const std::shared_ptr<Obstacle> &obstacleK,
                                               const std::shared_ptr<Obstacle> &obstacleP) {
    double vReqLane{
        requiredVelocity(obstacleK->getOccupiedLanelets(world->getRoadNetwork(), timeStep),
                         world->getRoadNetwork()->extractTrafficSignIDForCountry(TrafficSignTypes::MIN_SPEED))};
    return vReqLane <= obstacleK->getStateByTimeStep(timeStep)->getVelocity();
}

double RequiredSpeedPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                const std::shared_ptr<Obstacle> &obstacleP) {
    // TODO add robust mode
    throw std::runtime_error("RequiredSpeedPredicate does not support robust evaluation!");
}

Constraint RequiredSpeedPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                        const std::shared_ptr<Obstacle> &obstacleK,
                                                        const std::shared_ptr<Obstacle> &obstacleP) {
    // TODO add constraint mode
    throw std::runtime_error("RequiredSpeedPredicate does not support constraint evaluation!");
}