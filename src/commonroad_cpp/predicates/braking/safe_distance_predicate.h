//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "commonroad_cpp/predicates/commonroad_predicate.h"

/**
 * Predicate for the safe distance between two vehicles.
 */
class SafeDistancePredicate : public CommonRoadPredicate {
  public:
    /**
     * Constructor for SafeDistancePredicate.
     */
    SafeDistancePredicate();

    /**
     * Safe distance between the kth and the pth obstacle using objects.
     *
     * @param velocityK Velocity of the kth obstacle [m/s].
     * @param velocityP Velocity of the pth obstacle [m/s].
     * @param minAccelerationK Minimum (longitudinal) acceleration of the kth obstacle [m/s^2].
     * @param minAccelerationP Minimum (longitudinal) acceleration of the pth obstacle [m/s^2].
     * @param tReact Reaction time of the kth obstacle [s].
     * @return Safe distance [m].
     */
    static double computeSafeDistance(double velocityK, double velocityP, double minAccelerationK,
                                      double minAccelerationP, double tReact);

    /**
     * Boolean evaluation of predicate using objects.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @return Boolean indicating satisfaction of the predicate.
     */
    bool booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                           const std::shared_ptr<Obstacle> &obstacleK,
                           const std::shared_ptr<Obstacle> &obstacleP) override;

    /**
     * Boolean evaluation of predicate using parameter values.
     *
     * @param lonPosK Longitudinal position in the curvilinear coordinate system of the kth vehicle [m].
     * @param lonPosP Longitudinal position in the curvilinear coordinate system of the pth vehicle [m].
     * @param velocityK Velocity of the kth vehicle [m/s].
     * @param velocityP Velocity of the pth vehicle [m/s].
     * @param minAccelerationK Minimum (longitudinal) acceleration of the kth obstacle [m/s^2].
     * @param minAccelerationP Minimum (longitudinal) acceleration of the pth obstacle [m/s^2].
     * @param tReact Reaction time of the kth obstacle [s].
     * @param lengthK Length of the kth obstacle [m].
     * @param lengthP Length of the pth obstacle [m].
     * @return Boolean indicating satisfaction of the predicate.
     */
    static bool booleanEvaluation(double lonPosK, double lonPosP, double velocityK, double velocityP,
                                  double minAccelerationK, double minAccelerationP, double tReact, double lengthK,
                                  double lengthP);

    /**
     * Constraint evaluation of predicate using objects.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @return Constraints defined by the predicate.
     */
    Constraint constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                    const std::shared_ptr<Obstacle> &obstacleK,
                                    const std::shared_ptr<Obstacle> &obstacleP) override;

    /**
     * Constraint evaluation of predicate using parameter values.
     *
     * @param lonPosP Longitudinal position in the curvilinear coordinate system of the pth vehicle [m].
     * @param velocityK Velocity of the kth vehicle [m/s].
     * @param velocityP Velocity of the pth vehicle [m/s].
     * @param minAccelerationK Minimum (longitudinal) acceleration of the kth obstacle [m/s^2].
     * @param minAccelerationP Minimum (longitudinal) acceleration of the pth obstacle [m/s^2].
     * @param tReact Reaction time of the kth obstacle [s].
     * @param lengthK Length of the kth obstacle [m].
     * @param lengthP Length of the pth obstacle [m].
     * @return Constraints defined by the predicate.
     */
    static Constraint constraintEvaluation(double lonPosP, double velocityK, double velocityP, double minAccelerationK,
                                           double minAccelerationP, double tReact, double lengthK, double lengthP);

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
                            const std::shared_ptr<Obstacle> &obstacleK,
                            const std::shared_ptr<Obstacle> &obstacleP) override;

    /**
     * Robustness evaluation of predicate using parameter values.
     *
     * @param velocityK Velocity of the kth vehicle [m/s].
     * @param velocityP Velocity of the pth vehicle [m/s].
     * @param minAccelerationK Minimum (longitudinal) acceleration of the kth obstacle [m/s^2].
     * @param minAccelerationP Minimum (longitudinal) acceleration of the pth obstacle [m/s^2].
     * @param tReact Reaction time of the kth obstacle [s].
     * @param lengthK Length of the kth obstacle [m].
     * @param lengthP Length of the pth obstacle [m].
     * @return Real value indicating robustness of the predicate.
     */
    static double robustEvaluation(double lonPosK, double lonPosP, double velocityK, double velocityP,
                                   double minAccelerationK, double minAccelerationP, double tReact, double lengthK,
                                   double lengthP);
};
