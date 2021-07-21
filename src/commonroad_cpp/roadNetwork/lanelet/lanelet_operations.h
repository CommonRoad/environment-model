//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "../road_network.h"
#include "commonroad_cpp/auxiliaryDefs/types_and_definitions.h"
#include "lane.h"
#include "lanelet.h"

namespace lanelet_operations {
/**
 * Matches a string to a lanelet type
 *
 * @param string for which lanelet type should be extracted
 * @return lanelet type which corresponds to string or unknown type if string does not match
 */
LaneletType matchStringToLaneletType(const std::string &type);

/**
 * Matches a string to a line marking
 *
 * @param string for which line marking should be extracted
 * @return line marking which corresponds to string or unknown type if string does not match
 */
LineMarking matchStringToLineMarking(const std::string &type);

/**
 * Combines a lanelet and all its successors to lanes. For each successor lanelet a new lane is created. The algorithm
 * stops when the lane reaches a length given by the parameter fov.
 *
 * @param curLanelet Lanelet which is currently at the end of list and for which successors should be added.
 * @param fov Field of view defining length of lane.
 * @param containedLanelets List of contained lanelets in lane. Required for recursive call.
 * @return List containing a list of lanelets contained in lane.
 */
std::vector<std::vector<std::shared_ptr<Lanelet>>>
combineLaneletAndSuccessorsToLane(const std::shared_ptr<Lanelet> &curLanelet, double fov = 250,
                                  std::vector<std::shared_ptr<Lanelet>> containedLanelets = {});

/**
 * Combines a lanelet and all its predecessors to lanes. For each predecessor lanelet a new lane is created. The
 * algorithm stops when the lane reaches a length given by the parameter fov.
 *
 * @param curLanelet Lanelet which is currently at the end of list and for which predecessors should be added.
 * @param fov Field of view defining length of lane.
 * @param containedLanelets List of contained lanelets in lane. Required for recursive call.
 * @return List containing a list of lanelets contained in lane.
 */
std::vector<std::vector<std::shared_ptr<Lanelet>>>
combineLaneletAndPredecessorsToLane(const std::shared_ptr<Lanelet> &curLanelet, double fov = 250,
                                    std::vector<std::shared_ptr<Lanelet>> containedLanelets = {});

/**
 * Creates lanes which are originating from given set of lanelets.
 *
 * @param initialLanelets Initial lanelets based on which lanelets should be created.
 * @param idCounter Starting ID of new lanes.
 * @param roadNetwork Pointer to road network.
 * @param fov Field of view which defines length of lanes.
 * @return List of pointers to lanes originating from given lanelets.
 */
std::vector<std::shared_ptr<Lane>>
createLanesBySingleLanelets(const std::vector<std::shared_ptr<Lanelet>> &initialLanelets,
                            const std::shared_ptr<size_t> &idCounter, const std::shared_ptr<RoadNetwork> &roadNetwork,
                            double fov = 250);

/**
 * Creates lane objects given set of lanelets which form lane.
 *
 * @param containedLanelets List of pointers to lanelets.
 * @param newId ID of new lane.
 * @return Pointer to new lane.
 */
std::shared_ptr<Lane> createLaneByContainedLanelets(const std::vector<std::shared_ptr<Lanelet>> &containedLanelets,
                                                    size_t newId);

/**
 * Extracts all lanelets right of a given lanelet in the same direction.
 *
 * @param lanelet Pointer to reference lanelet.
 * @return List of pointers to lanelets.
 */
std::vector<std::shared_ptr<Lanelet>> laneletsRightOfLanelet(std::shared_ptr<Lanelet> lanelet);

/**
 * Extracts all lanelets left of a given lanelet in the same direction.
 *
 * @param lanelet Pointer to reference lanelet.
 * @return List of pointers to lanelets.
 */
std::vector<std::shared_ptr<Lanelet>> laneletsLeftOfLanelet(std::shared_ptr<Lanelet> lanelet);

/**
 * Extracts all adjacent lanelets in the same direction to given lanelet.
 *
 * @param lanelet Pointer to reference lanelet.
 * @return List of pointers to lanelets.
 */
std::vector<std::shared_ptr<Lanelet>> adjacentLanelets(const std::shared_ptr<Lanelet> &lanelet);

/**
 * Evaluates whether occupied lanelets are part of two adjacent lanes.
 * @param laneOne Pointer to first lane.
 * @param laneTwo Pointer to second lane.
 * @param relevantLanelets Lanelets which should be evaluated, e.g., occupied lanelets.
 * @return Boolean indicating whether lanelets part of two adjacent lanes.
 */
bool adjacentLanes(const std::shared_ptr<Lane> &laneOne, const std::shared_ptr<Lane> &laneTwo,
                   const std::vector<std::shared_ptr<Lanelet>> &relevantLanelets);

} // namespace lanelet_operations