//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include <memory>

#include "commonroad_cpp/auxiliaryDefs/structs.h"
#include "commonroad_cpp/auxiliaryDefs/timer.h"

#include "predicate_config.h"
#include <omp.h>

class Obstacle;
class World;

struct OptionalPredicateParameters {
    OptionalPredicateParameters() = default;
    OptionalPredicateParameters(std::vector<TrafficSignTypes> signType, std::vector<LaneletType> laneletType,
                                std::vector<TurningDirection> turningDirection);
    OptionalPredicateParameters(std::vector<TrafficSignTypes> signType);
    OptionalPredicateParameters(std::vector<LaneletType> laneletType);
    OptionalPredicateParameters(std::vector<TurningDirection> turningDirection);
    std::vector<TrafficSignTypes> signType;
    std::vector<LaneletType> laneletType;
    std::vector<TurningDirection> turningDirection;
};

/**
 * Interface for a predicate.
 */
class CommonRoadPredicate {
  public:
    /**
     * Default constructor for predicate class without parameter.
     *
     * @param vehicleDependent Boolean indicating whether predicate depends only on one vehicle
     */
    CommonRoadPredicate(bool vehicleDependent);

    virtual ~CommonRoadPredicate();

    CommonRoadPredicate(const CommonRoadPredicate &) = delete;
    CommonRoadPredicate(CommonRoadPredicate &) = default;

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
    virtual bool
    booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
                      const std::shared_ptr<Obstacle> &obstacleP,
                      const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters = {}) = 0;

    /**
     * Virtual function for the robustness evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @return Real value indicating robustness of the predicate.
     */
    virtual double
    robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
                     const std::shared_ptr<Obstacle> &obstacleP,
                     const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters = {}) = 0;

    /**
     * Virtual function for the constraint evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @return Constraints defined by the predicate.
     */
    virtual Constraint
    constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                         const std::shared_ptr<Obstacle> &obstacleK, const std::shared_ptr<Obstacle> &obstacleP,
                         const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters = {}) = 0;

    /**
     * Function for the statistical evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     */
    bool
    statisticBooleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                               const std::shared_ptr<Obstacle> &obstacleK,
                               const std::shared_ptr<Obstacle> &obstacleP = {},
                               const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters = {});

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

    /**
     * Getter for statistics of predicate.
     *
     * @return Statistics of predicate.
     */
    const PredicateStatistics &getStatistics() const;

    /**
     * Getter for timer of predicate.
     *
     * @return Timer of predicate.
     */
    const Timer &getEvaluationTimer() const;

    /**
     * Returns whether predicate is vehicle dependent.
     *
     * @return Boolean indicating whether predicate is vehicle dependent.
     */
    bool isVehicleDependent() const;

    /**
     * Resets predicate statistic.
     */
    void resetStatistics();

  protected:
    PredicateParameters parameters; //**< Struct containing parameters of all predicates. */
    Timer evaluationTimer;          //**< Time measuring object for predicates. */
    PredicateStatistics statistics; //**< Struct storing statistics of a predicate, e.g., average computation
                                    // time,  number of calls, etc. */
    const bool vehicleDependent; //**< Boolean indicating whether predicate depends on one specific obstacle or two. */
    omp_lock_t writelock;        //**< omp lock for statisticBooleanEvaluation*/
};

extern std::map<std::string, std::shared_ptr<CommonRoadPredicate>> predicates; //**< List of all predicates **/
