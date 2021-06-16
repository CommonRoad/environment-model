//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "commonroad_cpp/auxiliaryDefs/timer.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include "predicate_parameters.h"

/**
 * Interface for a predicate.
 */
class CommonRoadPredicate {
  public:
    /**
     * Constructor for predicate class.
     *
     * @param parameters Struct containing parameters of all predicates.
     */
    explicit CommonRoadPredicate(const PredicateParameters &parameters);

    /**
     * Default constructor for predicate class without parameter.
     *
     * @param parameters Struct containing parameters of all predicates.
     */
    CommonRoadPredicate();

    /**
     * Virtual function for the boolean evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @param obstacles Pointers to all obstacles. This is an optional parameter.
     * @return Boolean indicating satisfaction of the predicate.
     */
    virtual bool booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                   const std::shared_ptr<Obstacle> &obstacleK,
                                   const std::shared_ptr<Obstacle> &obstacleP) = 0;

    /**
     * Virtual function for the robustness evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @return Real value indicating robustness of the predicate.
     */
    virtual double robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                    const std::shared_ptr<Obstacle> &obstacleK,
                                    const std::shared_ptr<Obstacle> &obstacleP) = 0;

    /**
     * Virtual function for the constraint evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @return Constraints defined by the predicate.
     */
    virtual Constraint constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                            const std::shared_ptr<Obstacle> &obstacleK,
                                            const std::shared_ptr<Obstacle> &obstacleP) = 0;

    /**
     * Function for the statistical evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     */
    bool statisticBooleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                    const std::shared_ptr<Obstacle> &obstacleK,
                                    const std::shared_ptr<Obstacle> &obstacleP = {});

    /**
     * Getter for parameters.
     *
     * @return Struct containing all parameters.
     */
    [[nodiscard]] const PredicateParameters &getParameters() const;

    /**
     * Setter for parameters.
     *
     * @param parameters Struct containing all parameters.
     */
    void setParameters(const PredicateParameters &parameters);

    const PredicateStatistics &getStatistics() const;

    const Timer &getEvaluationTimer() const;

  protected:
    PredicateParameters parameters; //**< Struct containing parameters of all predicates. */
    Timer evaluationTimer;
    PredicateStatistics statistics; //**< Struct storing statistics of a predicate, e.g., average computation
                                    // time,  number of calls, etc. */
};
