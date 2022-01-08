//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "lane_based_orientation_similar_predicate.h"
#include "../../geometry/geometric_operations.h"
#include "../../world.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include <stdexcept>

bool LaneBasedOrientationSimilarPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                             const std::shared_ptr<Obstacle> &obstacleP,
                                                             const std::shared_ptr<Obstacle> &obstacleK) {
    return std::abs(geometric_operations::subtractOrientations(
               obstacleK->getCurvilinearOrientation(timeStep,
                                                    obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)),
               obstacleP->getCurvilinearOrientation(world->getRoadNetwork(), timeStep))) <
           parameters.laneMatchingOrientation;
}

Constraint LaneBasedOrientationSimilarPredicate::constraintEvaluation(size_t timeStep,
                                                                      const std::shared_ptr<World> &world,
                                                                      const std::shared_ptr<Obstacle> &obstacleP,
                                                                      const std::shared_ptr<Obstacle> &obstacleK) {
    throw std::runtime_error("Lane Based Orientation Similar does not support constraint evaluation!");
}

double LaneBasedOrientationSimilarPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                              const std::shared_ptr<Obstacle> &obstacleP,
                                                              const std::shared_ptr<Obstacle> &obstacleK) {
    throw std::runtime_error("Lane Based Orientation Similar does not support robust evaluation!");
}
LaneBasedOrientationSimilarPredicate::LaneBasedOrientationSimilarPredicate() : CommonRoadPredicate(true) {}
