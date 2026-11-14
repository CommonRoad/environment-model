#pragma once

#include "commonroad_cpp/predicates/lane/in_same_lane_predicate.h"

/**
 * Must-version of in_same_lane for set-based predictions. For the pth vehicle: all behaviors covered by the occupancy
 * share a lane with the kth vehicle, i.e., all lanelets occupied by the occupancy belong to lanes of the kth vehicle.
 * For the kth vehicle, the predicate is false, since its lanes are not determined for all covered behaviors.
 */
class InSameLaneMustPredicate : public InSameLanePredicate {
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
                           const std::shared_ptr<Obstacle> &obstacleK, const std::shared_ptr<Obstacle> &obstacleP,
                           const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                           bool setBased = false) override;
};
