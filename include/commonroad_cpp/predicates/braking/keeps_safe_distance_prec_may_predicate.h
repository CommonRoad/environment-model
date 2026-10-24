#pragma once

#include "commonroad_cpp/predicates/braking/keeps_safe_distance_prec_predicate.h"

/**
 * May-version of keeps_safe_distance_prec for a set-based prediction of the pth vehicle: at least one behavior
 * covered by the occupancy keeps the safe distance. Covered behaviors behind the front of the kth vehicle satisfy the
 * predicate, so it holds if any part of the occupancy is behind it; otherwise, the front of the occupancy and the
 * highest velocity of the covered behaviors are used (the robustness increases with both).
 */
class KeepsSafeDistancePrecMayPredicate : public KeepsSafeDistancePrecPredicate {
  public:
    /**
     * Robustness evaluation of the predicate; for set-based predictions, the may-version is used.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle.
     * @param additionalFunctionParameters Additional parameters.
     * @param setBased Boolean indicating whether set-based evaluation should be used.
     * @return Real value indicating robustness of the predicate.
     */
    double robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                            const std::shared_ptr<Obstacle> &obstacleK, const std::shared_ptr<Obstacle> &obstacleP,
                            const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                            bool setBased = false) override;
};
