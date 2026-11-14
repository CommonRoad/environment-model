#pragma once

#include "commonroad_cpp/predicates/lane/in_single_lane_predicate.h"

/**
 * Must-version of in_single_lane for a set-based prediction of the kth vehicle: all behaviors covered by the occupancy
 * are in a single lane, i.e., the lanelets occupied by the occupancy belong to a single lane (each covered behavior
 * occupies some of them). This equals the set-based evaluation of in_single_lane, which evaluates the occupancy
 * without passing setBased, but uses the set-based cache of the occupied lanelets.
 */
class InSingleLaneMustPredicate : public InSingleLanePredicate {
  public:
    /**
     * Boolean evaluation of the predicate; for set-based predictions, the version described above is used.
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
                           const std::shared_ptr<Obstacle> &obstacleK, const std::shared_ptr<Obstacle> &obstacleP = {},
                           const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                           bool setBased = false) override;
};
