//
// Created by Sebastian Maierhofer and Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "interstate_broad_enough_predicate.h"
#include "../../roadNetwork/lanelet/lanelet_operations.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/position/in_front_of_predicate.h"
#include <commonroad_cpp/world.h>

bool InterstateBroadEnoughPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP,
                                                       OptionalPredicateParameters additionalFunctionParameters) {
    double obsK_x = obstacleK->getStateByTimeStep(timeStep)->getXPosition();
    double obsK_y = obstacleK->getStateByTimeStep(timeStep)->getYPosition();
    auto occupied_lanelets = world->getRoadNetwork()->findLaneletsByPosition(obsK_x, obsK_y);
    return std::all_of(occupied_lanelets.begin(), occupied_lanelets.end(),
                       [obsK_x, obsK_y, world, this](const std::shared_ptr<Lanelet> &lanelet) {
                           return lanelet_operations::roadWidth(lanelet, obsK_x, obsK_y) >
                                  parameters.minInterstateWidth;
                       });
}

Constraint InterstateBroadEnoughPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                                const std::shared_ptr<Obstacle> &obstacleP,
                                                                OptionalPredicateParameters additionalFunctionParameters) {
    throw std::runtime_error("Interstate Broad Enough Predicate does not support constraint evaluation!");
}

double InterstateBroadEnoughPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                        const std::shared_ptr<Obstacle> &obstacleK,
                                                        const std::shared_ptr<Obstacle> &obstacleP,
                                                        OptionalPredicateParameters additionalFunctionParameters) {
    throw std::runtime_error("Interstate Broad Enough Traffic Predicate does not support robust evaluation!");
}

InterstateBroadEnoughPredicate::InterstateBroadEnoughPredicate() : CommonRoadPredicate(false) {}