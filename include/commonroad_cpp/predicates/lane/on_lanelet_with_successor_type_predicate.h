#pragma once

#include "commonroad_cpp/predicates/commonroad_predicate.h"

/**
 * Evaluates whether a vehicle is on a lanelet of a given type or any successor lanelet of the currently occupied lanes
 * is of that type. Determination within a distance specified in the config parameters.
 */
class OnLaneletWithSuccessorTypePredicate : public CommonRoadPredicate {
  public:
    /**
     * Constructor for OnLaneletWithSuccessorTypePredicate
     */
    OnLaneletWithSuccessorTypePredicate();

    /**
     * Boolean evaluation of predicate using objects.
     *
     * @param successor Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @return Boolean indicating satisfaction of the predicate.
     */
    bool booleanEvaluation(size_t successor, const std::shared_ptr<World> &world,
                           const std::shared_ptr<Obstacle> &obstacleK, const std::shared_ptr<Obstacle> &obstacleP = {},
                           const std::vector<std::string> &additionalFunctionParameters = {}) override;

    /**
     * Constraint evaluation of predicate using objects. (Currently not supported for this predicate)
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle.
     * @return Constraints defined by the predicate.
     */
    double robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                            const std::shared_ptr<Obstacle> &obstacleK, const std::shared_ptr<Obstacle> &obstacleP = {},
                            const std::vector<std::string> &additionalFunctionParameters = {}) override;

    /**
     * Robustness evaluation of predicate using objects. (Currently not supported for this predicate)
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @return Real value indicating robustness of the predicate.
     */
    Constraint constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                    const std::shared_ptr<Obstacle> &obstacleK,
                                    const std::shared_ptr<Obstacle> &obstacleP = {},
                                    const std::vector<std::string> &additionalFunctionParameters = {}) override;
};
