//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "commonroad_cpp/predicates/commonroad_predicate.h"

/**
 * Braking with given acceleration possible at an intersection incoming.
 */
class BrakingAtIntersectionPossiblePredicate : public CommonRoadPredicate {
  public:
    /**
     * Constructor for BrakingWithAccelerationPossible.
     */
    BrakingAtIntersectionPossiblePredicate();

    /**
     * Boolean evaluation of predicate.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @return Constraints defined by the predicate.
     */
    bool
    booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
                      const std::shared_ptr<Obstacle> &obstacleP = {},
                      const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters = {}) override;

    /**
     * Constraint evaluation of predicate.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @return Constraints defined by the predicate.
     */
    Constraint constraintEvaluation(
        size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
        const std::shared_ptr<Obstacle> &obstacleP = {},
        const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters = {}) override;

    /**
     * Robustness evaluation of predicate.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @param additionalFunctionParameters Additional parameters.
     * @return Real value indicating robustness of the predicate.
     */
    double
    robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
                     const std::shared_ptr<Obstacle> &obstacleP = {},
                     const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters = {}) override;
};
