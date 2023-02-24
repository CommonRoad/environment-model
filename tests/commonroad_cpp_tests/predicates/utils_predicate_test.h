//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

class RoadNetwork;

namespace utils_predicate_test {
std::shared_ptr<RoadNetwork>
create_road_network(const std::set<LaneletType> &laneletTypeLaneletOne = {LaneletType::interstate},
                    const std::set<LaneletType> &laneletTypeLaneletTwo = {LaneletType::interstate});
std::shared_ptr<RoadNetwork> create_road_network_2();
std::shared_ptr<RoadNetwork> create_road_network_3();
std::shared_ptr<RoadNetwork> create_road_network_with_2_successors(
    const std::set<LaneletType> &laneletTypeRight = {LaneletType::intersection},
    const std::set<LaneletType> &laneletTypeLeft = {LaneletType::intersection},
    const std::set<LaneletType> &laneletTypeSuccessorRight = {LaneletType::intersection},
    const std::set<LaneletType> &laneletTypeSuccessorLeft = {LaneletType::intersection});
std::shared_ptr<RoadNetwork>
create_road_network_with_circle(const std::set<LaneletType> &laneletTypeBottom = {LaneletType::urban},
                                const std::set<LaneletType> &laneletTypeLeft = {LaneletType::urban},
                                const std::set<LaneletType> &laneletTypeRight = {LaneletType::urban},
                                const std::set<LaneletType> &laneletTypeTop = {LaneletType::urban});
std::shared_ptr<RoadNetwork> create_narrow_road_network(double width);
std::shared_ptr<RoadNetwork> create_road_network_users(const std::set<ObstacleType> &userOneWayLanelet,
                                                       const std::set<ObstacleType> &userBidirectionalLanelet);
} // namespace utils_predicate_test