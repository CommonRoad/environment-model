//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "commonroad_cpp/predicates/commonroad_predicate.h"

/**
 *  Evaluates if a vehicle has an unobstructed view over the intersection.
 */
class UnobstructedIntersectionViewPredicate : public CommonRoadPredicate {
  public:
    /**
     * Constructor for UnobstructedIntersectionViewPredicate
     */
    UnobstructedIntersectionViewPredicate();

    /**
     * Boolean evaluation of predicate using objects.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @return Boolean indicating satisfaction of the predicate.
     */
    bool
    booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
                      const std::shared_ptr<Obstacle> &obstacleP = {},
                      const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters = {}) override;

    /**
     * Constraint evaluation of predicate using objects. (Currently not supported for this predicate)
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle.
     * @return Constraints defined by the predicate.
     */
    double
    robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
                     const std::shared_ptr<Obstacle> &obstacleP = {},
                     const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters = {}) override;

    /**
     * Robustness evaluation of predicate using objects. (Currently not supported for this predicate)
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @return Real value indicating robustness of the predicate.
     */
    Constraint constraintEvaluation(
        size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
        const std::shared_ptr<Obstacle> &obstacleP = {},
        const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters = {}) override;
};
