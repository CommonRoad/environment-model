#pragma once

#include "commonroad_cpp/predicates/commonroad_predicate.h"

/**
 * Predicate for causing braking of a vehicle in an intersection.
 */
class CausesBrakingIntersectionPredicate : public CommonRoadPredicate {
  public:
    /**
     * Constructor for UnnecessaryBrakingPredicate.
     */
    CausesBrakingIntersectionPredicate();

    /**
     * Boolean evaluation of predicate.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @param additionalFunctionParameters Additional parameters.
     * @param setBased Boolean indicating whether set-based evaluation should be used.
     * @return Constraints defined by the predicate.
     */
    bool booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                           const std::shared_ptr<Obstacle> &obstacleK, const std::shared_ptr<Obstacle> &obstacleP = {},
                           const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                           bool setBased = false) override;

    /**
     * Constraint evaluation of predicate.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @param additionalFunctionParameters Additional parameters.
     * @param setBased Boolean indicating whether set-based evaluation should be used.
     * @return Constraints defined by the predicate.
     */
    Constraint constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                    const std::shared_ptr<Obstacle> &obstacleK,
                                    const std::shared_ptr<Obstacle> &obstacleP = {},
                                    const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                                    bool setBased = false) override;

    /**
     * Robustness evaluation of predicate.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @param additionalFunctionParameters Additional parameters.
     * @param setBased Boolean indicating whether set-based evaluation should be used.
     * @return Real value indicating robustness of the predicate.
     */
    double robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                            const std::shared_ptr<Obstacle> &obstacleK, const std::shared_ptr<Obstacle> &obstacleP = {},
                            const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                            bool setBased = false) override;

  protected:
    /**
     * Evaluates the must- or may-version of the predicate for set-based predictions; used by the predicates with the
     * suffix _must and _may. Without an occupancy to evaluate, the base evaluation is used.
     *
     *
     * For a set-based prediction of the pth vehicle, the must-version requires that all covered behaviors brake and
     * that the whole occupancy lies within a lane with all covered distances to an intersection point within the
     * braking interval; the may-version requires that some covered behavior brakes and that the longitudinal extent of
     * the occupancy clipped to a lane overlaps the braking interval. For a set-based prediction of the kth vehicle, the
     * condition has to hold for every (must) or some (may) occupied lane.
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
