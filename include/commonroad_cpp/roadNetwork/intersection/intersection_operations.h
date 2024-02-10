#pragma once

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

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

/**
 * Gets the intersection on which the obstacle is currently.
 * @param timeStep Time step of interest.
 * @param obs Pointer to obstacle.
 * @param roadNetwork Pointer to road network.
 * @return Boolean indicating whether incoming is occupied by obstacle.
 */
std::shared_ptr<Intersection> currentIntersection(size_t timeStep, const std::shared_ptr<World> &world,
                                                  const std::shared_ptr<Obstacle> &obstacleK);

/**
 * Gets the incoming from the intersection on which the obstacle is currently.
 * @param timeStep Time step of interest.
 * @param obs Pointer to obstacle.
 * @param roadNetwork Pointer to road network.
 * @return Boolean indicating whether incoming is occupied by obstacle.
 */
std::shared_ptr<IncomingGroup> currentIncoming(size_t timeStep, const std::shared_ptr<World> &world,
                                               const std::shared_ptr<Obstacle> &obs);

} // namespace intersection_operations
