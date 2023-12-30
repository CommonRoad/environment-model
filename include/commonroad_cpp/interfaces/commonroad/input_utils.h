//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <string>
#include <tuple>
#include <vector>

#include <commonroad_cpp/interfaces/commonroad/xml_reader.h>

class RoadNetwork;

using Scenario = std::tuple<std::vector<std::shared_ptr<Obstacle>>, std::shared_ptr<RoadNetwork>, double>;

namespace InputUtils {

/**
 * Loads and sets up CR scenario. The file can be encoded in xml or protobuf format.
 *
 * @param filePath Path to CommonRoad file
 * @return Tuple of obstacles, roadNetwork, and time step size
 */
Scenario getDataFromCommonRoad(const std::string &filePath);


} // namespace InputUtils
