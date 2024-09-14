#pragma once

#include "../commonroad_predicate.h"

/**
 * Predicate which checks if the kth vehicle is oriented to the left or right based on its reference lane reference.
 */
class LaneBasedOrientationSidePredicate : public CommonRoadPredicate {
  public:
    /**
     * Constructor for LaneBasedOrientationInDirectionPredicate.
     */
    LaneBasedOrientationSidePredicate();

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
                           const std::shared_ptr<Obstacle> &obstacleK, const std::shared_ptr<Obstacle> &obstacleP,
                           const std::vector<std::string> &additionalFunctionParameters = {}) override;

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
                            const std::shared_ptr<Obstacle> &obstacleK, const std::shared_ptr<Obstacle> &obstacleP,
                            const std::vector<std::string> &additionalFunctionParameters = {}) override;

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
                                    const std::shared_ptr<Obstacle> &obstacleP,
                                    const std::vector<std::string> &additionalFunctionParameters = {}) override;
};
