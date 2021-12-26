//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

class RoadNetwork;

namespace utils_predicate_test {
std::shared_ptr<RoadNetwork> create_road_network();
std::shared_ptr<RoadNetwork> create_road_network_2();
std::shared_ptr<RoadNetwork> create_road_network_3();
} // namespace utils_predicate_test