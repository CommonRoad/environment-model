//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "commonroad_cpp/auxiliaryDefs/types_and_definitions.h"
#include "../road_network.h"
#include "lane.h"
#include "lanelet.h"

namespace lanelet_operations {
/**
 * Matches a string to a driving direction
 *
 * @param string for which driving direction should be extracted
 * @return driving direction type which corresponds to string or unknown type if string does not match
 */
DrivingDirection matchStringToDrivingDirection(const std::string &type);

/**
 * Matches a driving direction to a string
 *
 * @param driving direction for which string should be extracted
 * @return string which corresponds to driving direction
 */
std::string matchDrivingDirectionToString(const DrivingDirection &type);

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
 * Iteratively concatenates lanelets with same lanelet type given a start lanelet and creates a lane out of them
 *
 * @param curLanelet start lanelet for which concatenation should be started
 * @param type specifies the lanelet type the concatenated lanelets should have in common
 * @return lane spanned by concatenated lanelets
 */
std::vector<std::vector<std::shared_ptr<Lanelet>>>
combineLaneletAndSuccessorsToLane(const std::shared_ptr<Lanelet> &curLanelet, double fov = 250,
                                  std::vector<std::shared_ptr<Lanelet>> containedLanelets = {});

std::vector<std::vector<std::shared_ptr<Lanelet>>>
combineLaneletAndPredecessorsToLane(const std::shared_ptr<Lanelet> &curLanelet, double fov = 250,
                                    std::vector<std::shared_ptr<Lanelet>> containedLanelets = {});

std::vector<std::shared_ptr<Lane>> createLanesBySingleLanelets(
    const std::vector<std::shared_ptr<Lanelet>> &initialLanelets, size_t &idCounter,
    std::shared_ptr<RoadNetwork> roadNetwork, double fov = 250);

std::shared_ptr<Lane> createLaneByContainedLanelets(const std::vector<std::shared_ptr<Lanelet>> &containedLanelets,
                                                    size_t newId);

std::shared_ptr<Lane> mergeLanes(std::shared_ptr<Lane> predecessorLane, std::shared_ptr<Lane> successorLane,
                                 size_t newId);

bool containsLaneletType(LaneletType type, std::set<LaneletType> baseTypesSet);

std::vector<size_t> extractIds(std::vector<std::shared_ptr<Lanelet>> lanelets);

std::vector<std::shared_ptr<Lanelet>> laneletsRightOfLanelet(std::shared_ptr<Lanelet> lanelet);

std::vector<std::shared_ptr<Lanelet>> laneletsLeftOfLanelet(std::shared_ptr<Lanelet> lanelet);

std::vector<std::shared_ptr<Lanelet>> adjacentLanelets(std::shared_ptr<Lanelet> lanelet);

bool adjacentLanes(std::shared_ptr<Lane> laneOne, std::shared_ptr<Lane> laneTwo,
                   std::vector<std::shared_ptr<Lanelet>> relevantLanelets);

} // namespace lanelet_operations