#pragma once

#include "commonroad_cpp/predicates/lane/in_single_lane_predicate.h"

/**
 * May-version of in_single_lane for a set-based prediction of the kth vehicle: at least one behavior covered by the
 * occupancy is in a single lane. The occupancy contains no information about the extent of the single covered
 * behaviors, so the predicate conservatively holds for occupancies.
 */
class InSingleLaneMayPredicate : public InSingleLanePredicate {
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
