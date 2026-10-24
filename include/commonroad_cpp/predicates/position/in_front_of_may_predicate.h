#pragma once

#include "commonroad_cpp/predicates/position/in_front_of_predicate.h"

/**
 * May-version of in_front_of for a set-based prediction of the kth vehicle: at least one behavior covered by the
 * occupancy is in front of the pth vehicle, i.e., the front of the occupancy is in front of the front of the pth
 * vehicle.
 */
class InFrontOfMayPredicate : public InFrontOfPredicate {
  public:
    /**
     * Robustness evaluation of the predicate; for set-based predictions, the may-version is used.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleP The pth obstacle.
     * @param obstacleK The kth obstacle.
     * @param additionalFunctionParameters Additional parameters.
     * @param setBased Boolean indicating whether set-based evaluation should be used.
     * @return Real value indicating robustness of the predicate.
     */
    double robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                            const std::shared_ptr<Obstacle> &obstacleP, const std::shared_ptr<Obstacle> &obstacleK,
                            const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                            bool setBased = false) override;
};
