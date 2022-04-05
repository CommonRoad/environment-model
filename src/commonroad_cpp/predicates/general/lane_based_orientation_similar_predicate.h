//
// Created by Sebastian Maierhofer and Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "commonroad_cpp/predicates/commonroad_predicate.h"

/**
 * Evaluates whether the kth vehicle is in front of the pth vehicle, withing the same lane, and drives in the same
 * direction.
 */
class LaneBasedOrientationSimilarPredicate : public CommonRoadPredicate {
  public:
    /**
     * Constructor for LaneBasedOrientationSimilarPredicate.
     */
    LaneBasedOrientationSimilarPredicate();

    /**
     * Boolean evaluation of predicate using objects.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle.
     * @return Boolean indicating satisfaction of the predicate.
     */
    bool booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                           const std::shared_ptr<Obstacle> &obstacleP, const std::shared_ptr<Obstacle> &obstacleK = {},
                           OptionalPredicateParameters additionalFunctionParameters = {}) override;

    /**
     * Constraint evaluation of predicate using objects.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle.
     * @return Constraints defined by the predicate.
     */
    Constraint constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                    const std::shared_ptr<Obstacle> &obstacleP,
                                    const std::shared_ptr<Obstacle> &obstacleK = {},
                                    OptionalPredicateParameters additionalFunctionParameters = {}) override;

    /**
     * Robustness evaluation of predicate using objects.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @return Real value indicating robustness of the predicate.
     */
    double robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                            const std::shared_ptr<Obstacle> &obstacleP, const std::shared_ptr<Obstacle> &obstacleK = {},
                            OptionalPredicateParameters additionalFunctionParameters = {}) override;
};
