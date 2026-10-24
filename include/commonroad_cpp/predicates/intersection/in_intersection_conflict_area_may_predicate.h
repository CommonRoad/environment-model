#pragma once

#include "commonroad_cpp/predicates/intersection/in_intersection_conflict_area_predicate.h"

/**
 * May-version of in_intersection_conflict_area for set-based predictions. For the kth vehicle, lanelets in its
 * driving direction are not excluded, since set-based predictions contain no orientation. For the pth vehicle, the
 * lanelets of any occupied lane are used.
 */
class InIntersectionConflictAreaMayPredicate : public InIntersectionConflictAreaPredicate {
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
