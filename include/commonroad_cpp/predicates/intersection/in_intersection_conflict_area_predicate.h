#pragma once

#include "commonroad_cpp/predicates/commonroad_predicate.h"

/**
 *  Evaluates if the kth obstacle is in the conflict area of the pth obstacle
 */
class InIntersectionConflictAreaPredicate : public CommonRoadPredicate {
  public:
    /**
     * Constructor for InIntersectionConflictAreaPredicate
     */
    InIntersectionConflictAreaPredicate();

    /**
     * Boolean evaluation of predicate using objects.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @param additionalFunctionParameters Additional parameters.
     * @param setBased Boolean indicating whether set-based evaluation should be used.
     * @return Boolean indicating satisfaction of the predicate.
     */
    bool booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                           const std::shared_ptr<Obstacle> &obstacleK, const std::shared_ptr<Obstacle> &obstacleP = {},
                           const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                           bool setBased = false) override;

    /**
     * Constraint evaluation of predicate using objects. (Currently not supported for this predicate)
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle.
     * @param additionalFunctionParameters Additional parameters.
     * @param setBased Boolean indicating whether set-based evaluation should be used.
     * @return Constraints defined by the predicate.
     */
    double robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                            const std::shared_ptr<Obstacle> &obstacleK, const std::shared_ptr<Obstacle> &obstacleP = {},
                            const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                            bool setBased = false) override;

    /**
     * Robustness evaluation of predicate using objects. (Currently not supported for this predicate)
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @param additionalFunctionParameters Additional parameters.
     * @param setBased Boolean indicating whether set-based evaluation should be used.
     * @return Real value indicating robustness of the predicate.
     */
    Constraint constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                    const std::shared_ptr<Obstacle> &obstacleK,
                                    const std::shared_ptr<Obstacle> &obstacleP = {},
                                    const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                                    bool setBased = false) override;

  protected:
    /**
     * Evaluates the must- or may-version of the predicate for set-based predictions; used by the predicates with the
     * suffix _must and _may. Without an occupancy to evaluate, the base evaluation is used.
     *
     *
     * For a set-based prediction of the kth vehicle, the must-version is false and the may-version skips the exclusion
     * of lanelets in the driving direction, since set-based predictions contain no orientation. For a set-based
     * prediction of the pth vehicle, the lanelets of all (must) or of any (may) occupied lane are used.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle.
     * @param setBased Boolean indicating whether set-based evaluation should be used.
     * @param must Boolean indicating whether the must-version (true) or the may-version (false) is evaluated.
     * @return Boolean indicating satisfaction of the predicate.
     */
    bool evaluateSetBased(size_t timeStep, const std::shared_ptr<World> &world,
                          const std::shared_ptr<Obstacle> &obstacleK, const std::shared_ptr<Obstacle> &obstacleP,
                          bool setBased, bool must);
};
