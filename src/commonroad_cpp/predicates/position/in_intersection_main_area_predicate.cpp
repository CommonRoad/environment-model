//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "in_intersection_main_area_predicate.h"

bool InIntersectionMainAreaPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                        const std::shared_ptr<Obstacle> &obstacleK,
                                                        const std::shared_ptr<Obstacle> &obstacleP) {
    auto lanelets{obstacleK->getOccupiedLanelets(world->getRoadNetwork(), timeStep)};
    for (const auto &la : lanelets) {
        if (std::any_of(la->getLaneletType().begin(), la->getLaneletType().end(),
                        [](LaneletType t) { return t == LaneletType::intersection; }))
            return true;
    }
    return false;
}

double InIntersectionMainAreaPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                         const std::shared_ptr<Obstacle> &obstacleK,
                                                         const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("InIntersectionMainAreaPredicate does not support robust evaluation!");
}

Constraint InIntersectionMainAreaPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                                 const std::shared_ptr<Obstacle> &obstacleP = {}) {
    throw std::runtime_error("InIntersectionMainAreaPredicate does not support constraint evaluation!");
}
