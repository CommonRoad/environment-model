#pragma once

#include "commonroad_cpp/predicates/commonroad_predicate.h"

#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"

/**
 *  Evaluates if a vehicle is close to intersection by distance.
 *  The distance is specified in the config parameters.
 */
class CloseToIntersectionPredicate : public CommonRoadPredicate {
  public:
    /**
      * Constructor for CloseToIntersectionPredicate
     */
    CloseToIntersectionPredicate();

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
