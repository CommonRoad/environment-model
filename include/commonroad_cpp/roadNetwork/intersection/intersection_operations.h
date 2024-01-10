#pragma once

#include "../../obstacle/obstacle.h"
#include "../road_network.h"

namespace intersection_operations {

/**
 * Evaluates whether obstacle occupies a incoming lanelet.
 * @param timeStep Time step of interest.
 * @param obs Pointer to obstacle.
 * @param roadNetwork Pointer to road network.
 * @return Boolean indicating whether incoming is occupied by obstacle.
 */
bool onIncoming(size_t timeStep, const std::shared_ptr<Obstacle> &obs, const std::shared_ptr<RoadNetwork> &roadNetwork);

/**
 *
 * @param letK
 * @param letP
 * @param roadNetwork
 * @return
 */
bool checkSameIncoming(const std::shared_ptr<Lanelet> &letK, const std::shared_ptr<Lanelet> &letP);

/**
 * Calculate left of given incomingGroup
 *
 * @param origin originGroup of interest
 */
void findLeftOf(const std::shared_ptr<IncomingGroup> &origin, const std::shared_ptr<RoadNetwork> &roadNetwork);

} // namespace intersection_operations
