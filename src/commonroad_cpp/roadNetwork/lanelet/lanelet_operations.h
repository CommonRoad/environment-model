//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "commonroad_cpp/auxiliaryDefs/types_and_definitions.h"
#include "lane.h"
#include "lanelet.h"

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
std::vector<std::shared_ptr<Lane>>
combineLaneletAndSuccessorsWithSameTypeToLane(const std::shared_ptr<Lanelet> &curLanelet,
                                              const Lanelet &curLaneLanelet = {},
                                              std::vector<std::shared_ptr<Lanelet>> containedLanelets = {});
