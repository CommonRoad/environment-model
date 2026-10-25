#pragma once

#include "commonroad_cpp/predicates/regulatory/has_priority_predicate.h"

/**
 * Must-version of has_priority for set-based predictions: all behaviors covered by the occupancies have the
 * priority relation. The priority of a vehicle is the lowest priority of the incoming lanelets it occupies; it is
 * undefined (and the predicate false) if it occupies no incoming lanelet. For an occupancy, every lanelet it occupies
 * has to be an incoming lanelet, and the priority bound over all covered behaviors is used: for the kth vehicle the
 * lowest priority of its lanelets, for the pth vehicle the highest priority of a single lanelet.
 */
class HasPriorityMustPredicate : public HasPriorityPredicate {
  public:
    /**
     * Boolean evaluation of the predicate; for set-based predictions, the must-version is used.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle.
     * @param additionalFunctionParameters Directions of the kth and pth vehicle.
     * @param setBased Boolean indicating whether set-based evaluation should be used.
     * @return Boolean indicating satisfaction of the predicate.
     */
    bool booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                           const std::shared_ptr<Obstacle> &obstacleK, const std::shared_ptr<Obstacle> &obstacleP,
                           const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                           bool setBased = false) override;
};
