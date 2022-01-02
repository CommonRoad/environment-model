//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

#include "../../roadNetwork/regulatoryElements/regulatory_elements_utils.h"
#include "keeps_lane_speed_limit_predicate.h"
#include "preserves_traffic_flow_predicate.h"

bool PreservesTrafficFlowPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                      const std::shared_ptr<Obstacle> &obstacleK,
                                                      const std::shared_ptr<Obstacle> &obstacleP) {
    double vMax{std::min({regulatory_elements_utils::speedLimitSuggested(
                              obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep),
                              world->getRoadNetwork()->extractTrafficSignIDForCountry(TrafficSignTypes::MAX_SPEED)),
                          regulatory_elements_utils::typeSpeedLimit(obstacleK->getObstacleType()),
                          EgoVehicleParameters().brakingSpeedLimit, EgoVehicleParameters().fovSpeedLimit,
                          EgoVehicleParameters().roadConditionSpeedLimit})};
    return (vMax - obstacleK->getStateByTimeStep(timeStep)->getVelocity()) < parameters.minVelocityDif;
}

double PreservesTrafficFlowPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("PreservesTrafficFlowPredicate does not support robust evaluation!");
}

Constraint PreservesTrafficFlowPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                               const std::shared_ptr<Obstacle> &obstacleK,
                                                               const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("PreservesTrafficFlowPredicate does not support constraint evaluation!");
}
PreservesTrafficFlowPredicate::PreservesTrafficFlowPredicate() : CommonRoadPredicate(false) {}
