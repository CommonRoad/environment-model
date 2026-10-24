#pragma once

#include "commonroad_cpp/predicates/lane/lane_based_orientation_similar_predicate.h"

/**
 * Must-version of lane_based_orientation_similar for a set-based prediction of the kth vehicle. Set-based
 * predictions contain no orientation, so every orientation is covered: no covered orientation is guaranteed to be
 * similar, i.e., the predicate is false.
 */
class LaneBasedOrientationSimilarMustPredicate : public LaneBasedOrientationSimilarPredicate {
  public:
    /**
     * Boolean evaluation of the predicate; for set-based predictions, the must-version is used.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleP The pth obstacle.
     * @param obstacleK The kth obstacle.
     * @param additionalFunctionParameters Additional parameters.
     * @param setBased Boolean indicating whether set-based evaluation should be used.
     * @return Boolean indicating satisfaction of the predicate.
     */
    bool booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                           const std::shared_ptr<Obstacle> &obstacleP, const std::shared_ptr<Obstacle> &obstacleK,
                           const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                           bool setBased = false) override;
};
