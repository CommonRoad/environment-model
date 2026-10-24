#pragma once

#include "commonroad_cpp/predicates/braking/causes_braking_intersection_predicate.h"

/**
 * May-version of causes_braking_intersection for set-based predictions. For the pth vehicle: some covered behavior
 * brakes, and the longitudinal extent of the occupancy clipped to a lane overlaps the braking interval. For the kth
 * vehicle: the condition holds for some lane occupied by the kth vehicle.
 */
class CausesBrakingIntersectionMayPredicate : public CausesBrakingIntersectionPredicate {
  public:
    /**
     * Boolean evaluation of the predicate; for set-based predictions, the may-version is used.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle.
     * @param additionalFunctionParameters Additional parameters.
     * @param setBased Boolean indicating whether set-based evaluation should be used.
     * @return Boolean indicating satisfaction of the predicate.
     */
    bool booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                           const std::shared_ptr<Obstacle> &obstacleK, const std::shared_ptr<Obstacle> &obstacleP,
                           const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                           bool setBased = false) override;
};
