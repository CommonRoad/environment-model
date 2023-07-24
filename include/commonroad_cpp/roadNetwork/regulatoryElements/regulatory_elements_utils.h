#pragma once

#include <set>

#include "../../obstacle/obstacle.h"
#include "../road_network.h"

namespace regulatory_elements_utils {

/**
 * Returns all active traffic lights relevant for an obstacle.
 *
 * @param timeStep Time Step of interest.
 * @param obs Pointer to obstacle.
 * @param roadNetwork Pointer to road network.
 * @return Set of pointers to traffic lights.
 */
std::set<std::shared_ptr<TrafficLight>> activeTrafficLights(size_t timeStep, const std::shared_ptr<Obstacle> &obs,
                                                            const std::shared_ptr<RoadNetwork> &roadNetwork);

/**
 * Evaluates whether obstacle is at red traffic light for defined turning direction.
 *
 * @param timeStep Time Step of interest.
 * @param obs Pointer to obstacle.
 * @param roadNetwork Pointer to road network.
 * @param turnDir Turning direction which should be considered.
 * @param tlState Traffic light state (color) which should be considered.
 * @return Boolean indicating whether obstacle is at red traffic light.
 */
bool atTrafficLightDirState(size_t timeStep, const std::shared_ptr<Obstacle> &obs,
                            const std::shared_ptr<RoadNetwork> &roadNetwork, TurningDirection turnDir,
                            TrafficLightState tlState);

/**
 * Computes applicable speed limit on provided lanelet.
 *
 * @param lanelet Pointer to lanelet which should be considered
 * @param signType Type of speed limit sign.
 * @return Speed limit [m/s]
 */
double speedLimit(const std::shared_ptr<Lanelet> &lanelet, const TrafficSignTypes &signType);

/**
 * Computes applicable speed limit on provided lanelets.
 *
 * @param lanelets List of pointers to lanelets which should be considered.
 * @param signType Type of speed limit sign.
 * @return Speed limit [m/s]
 */
double speedLimit(const std::vector<std::shared_ptr<Lanelet>> &lanelets, const TrafficSignTypes &signType);

/**
 * Computes applicable speed limit on provided lanelets and considers suggested speed limit.
 *
 * @param lanelets List of pointers to lanelets which should be considered.
 * @param signType Type of speed limit sign.
 * @return Speed limit [m/s]
 */
double speedLimitSuggested(const std::vector<std::shared_ptr<Lanelet>> &lanelets, const TrafficSignTypes &signType);

/**
 * Computes applicable required speed on provided lanelet.
 *
 * @param lanelet Pointer to lanelet which should be considered
 * @param signType Type of required speed sign.
 * @return Required speed [m/s]
 */
double requiredVelocity(const std::shared_ptr<Lanelet> &lanelet, const TrafficSignTypes &signType);

/**
 * Computes applicable required speed on provided lanelets.
 *
 * @param lanelets List of pointers to lanelets which should be considered.
 * @param signType Type of required speed sign.
 * @return Required speed [m/s]
 */
double requiredVelocity(const std::vector<std::shared_ptr<Lanelet>> &lanelets, const TrafficSignTypes &signType);

/**
 * Evaluates speed limit for a obstacle type. Currently, only the type speed limit for trucks is added.
 *
 * @param obstacleType Tye of relevant obstacle.
 * @return Speed limit [m/s]
 */
double typeSpeedLimit(ObstacleType obstacleType);

std::vector<std::string> getRelevantPrioritySignIDs();

std::shared_ptr<TrafficSignElement> extractPriorityTrafficSign(const std::shared_ptr<Lanelet> &lanelet);

int extractPriorityTrafficSign(const std::vector<std::shared_ptr<Lanelet>> &lanelets, TurningDirection dir);

int getPriority(size_t timeStep, const std::shared_ptr<RoadNetwork> &roadNetwork, const std::shared_ptr<Obstacle> &obs,
                TurningDirection dir);

/**
 * Matches national traffic sign ID to global CommonRoad ID.
 * @param trafficSignId National traffic sign ID string.
 * @param country Country.
 * @param country_string String representation of country.
 * @return CommonRoad traffic sign type object.
 */
TrafficSignTypes extractTypeFromNationalID(const std::string &trafficSignId, SupportedTrafficSignCountry country,
                                       const std::string &country_string);

} // namespace regulatory_elements_utils
