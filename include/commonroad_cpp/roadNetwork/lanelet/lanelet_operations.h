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
 * @param type for which lanelet type should be extracted
 * @return lanelet type which corresponds to string or unknown type if string does not match
 */
LaneletType matchStringToLaneletType(const std::string &type);

/**
 * Matches a string to a line marking
 *
 * @param type for which line marking should be extracted
 * @return line marking which corresponds to string or unknown type if string does not match
 */
LineMarking matchStringToLineMarking(const std::string &type);

/**
 * Extracts all lanelets right of a given lanelet in the same direction.
 *
 * @param lanelet Pointer to reference lanelet.
 * @param sameDirection Boolean indicating whether only lanelets in the same direction should be considered.
 * @return List of pointers to lanelets.
 */
std::vector<std::shared_ptr<Lanelet>> laneletsRightOfLanelet(std::shared_ptr<Lanelet> lanelet,
                                                             bool sameDirection = true);

/**
 * Extracts all lanelets left of a given lanelet in the same direction.
 *
 * @param lanelet Pointer to reference lanelet.
 * @param sameDirection Boolean indicating whether only lanelets in the same direction should be considered.
 * @return List of pointers to lanelets.
 */
std::vector<std::shared_ptr<Lanelet>> laneletsLeftOfLanelet(std::shared_ptr<Lanelet> lanelet,
                                                            bool sameDirection = true);

/**
 * Extracts all adjacent lanelets in the same direction to given lanelet including given lanelet.
 *
 * @param lanelet Pointer to reference lanelet.
 * @param sameDirection Boolean indicating whether only lanelets in the same direction should be considered.
 * @return List of pointers to lanelets.
 */
std::vector<std::shared_ptr<Lanelet>> adjacentLanelets(const std::shared_ptr<Lanelet> &lanelet,
                                                       bool sameDirection = true);

/**
 * Evaluates whether occupied lanelets are part of two directly adjacent lanes.
 *
 * @param laneOne Pointer to first lane.
 * @param laneTwo Pointer to second lane.
 * @param relevantLanelets Lanelets which should be evaluated, e.g., occupied lanelets. If only a single lane is
 * provided the algorithm returns false.
 * @return Boolean indicating whether lanelets part of two adjacent lanes.
 */
bool areLaneletsInDirectlyAdjacentLanes(const std::shared_ptr<Lane> &laneOne, const std::shared_ptr<Lane> &laneTwo,
                                        const std::vector<std::shared_ptr<Lanelet>> &relevantLanelets);

/**
 * Computes the width of a road considering all adjacent lanelets of a given lanelet.
 *
 * @param lanelet Lanelet based on which width of road should be calculated.
 * @param xPosition x-position for which width should be computed (must be valid position for provided lanelet)
 * @param yPosition y-position for which width should be computed (must be valid position for provided lanelet)
 * @return Width of lanelet at given position [m]
 */
double roadWidth(const std::shared_ptr<Lanelet> &lanelet, double xPosition, double yPosition);

/**
 * Extracts all active traffic lights from lanelet.
 *
 * @param lanelet Lanelet of interest.
 * @return List of traffic lights.
 */
std::vector<std::shared_ptr<TrafficLight>> activeTlsByLanelet(size_t timeStep, const std::shared_ptr<Lanelet> &lanelet);

/**
 * Checks whether a bicycleLane is next to a road without a gap inbetween.
 * @param lanlet Lanelet of interest
 * @return bool
 */
bool bicycleLaneNextToRoad(const std::shared_ptr<Lanelet> &lanelet);
} // namespace lanelet_operations
