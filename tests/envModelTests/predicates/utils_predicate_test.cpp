//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/auxiliaryDefs/traffic_signs.h>

#include "utils_predicate_test.h"

namespace utils_predicate_test {
std::shared_ptr<RoadNetwork> create_road_network() {
    const auto *trafficSignIDLookupTable = TrafficSignLookupTableByCountry.at(SupportedTrafficSignCountry::GERMANY);
    // max speed traffic sign
    size_t trafficSignIdOne = 200;
    std::vector<std::string> trafficSignElementOneValues{"50"};
    auto trafficSignElementsOne{std::vector<std::shared_ptr<TrafficSignElement>>{std::make_shared<TrafficSignElement>(
        trafficSignIDLookupTable->at(TrafficSignTypes::MAX_SPEED), trafficSignElementOneValues)}};
    auto vertexOne{vertex{0, 0}};
    auto trafficSignOne{std::make_shared<TrafficSign>(trafficSignIdOne, trafficSignElementsOne, vertexOne, false)};
    // required speed traffic sign
    size_t trafficSignIdTwo = 201;
    std::vector<std::string> trafficSignElementTwoValues{"10"};
    auto trafficSignElementsTwo{std::vector<std::shared_ptr<TrafficSignElement>>{std::make_shared<TrafficSignElement>(
        trafficSignIDLookupTable->at(TrafficSignTypes::MIN_SPEED), trafficSignElementTwoValues)}};
    auto vertTwo{vertex{0, 0}};
    auto trafficSignTwo{std::make_shared<TrafficSign>(trafficSignIdOne, trafficSignElementsTwo, vertTwo, false)};
    // right lanelet
    size_t idLaneletOne = 100;
    auto laneletTypeLaneletOne = std::set<LaneletType>{LaneletType::mainCarriageWay, LaneletType::interstate};
    auto userOneWayLaneletOne = std::set<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    auto userBidirectionalLaneletOne = std::set<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    auto leftBorderLaneletOne = std::vector<vertex>{
        vertex{0, 4},   vertex{10, 4},  vertex{20, 4},  vertex{30, 4},  vertex{40, 4},  vertex{50, 4},
        vertex{60, 4},  vertex{70, 4},  vertex{80, 4},  vertex{90, 4},  vertex{100, 4}, vertex{110, 4},
        vertex{120, 4}, vertex{130, 4}, vertex{140, 4}, vertex{150, 4}, vertex{160, 4}, vertex{170, 4}};
    auto rightBorderLaneletOne = std::vector<vertex>{
        vertex{0, 0},   vertex{10, 0},  vertex{20, 0},  vertex{30, 0},  vertex{40, 0},  vertex{50, 0},
        vertex{60, 0},  vertex{70, 0},  vertex{80, 0},  vertex{90, 0},  vertex{100, 0}, vertex{110, 0},
        vertex{120, 0}, vertex{130, 0}, vertex{140, 0}, vertex{150, 0}, vertex{160, 0}, vertex{170, 0}};
    auto laneletOne =
        std::make_shared<Lanelet>(Lanelet(idLaneletOne, leftBorderLaneletOne, rightBorderLaneletOne,
                                          laneletTypeLaneletOne, userOneWayLaneletOne, userBidirectionalLaneletOne));
    laneletOne->addTrafficSign(trafficSignOne);
    laneletOne->addTrafficSign(trafficSignTwo);

    // left lanelet
    size_t idLaneletTwo = 101;
    auto laneletTypeLaneletTwo = std::set<LaneletType>{LaneletType::mainCarriageWay, LaneletType::interstate};
    auto userOneWayLaneletTwo = std::set<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    auto userBidirectionalLaneletTwo = std::set<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    auto leftBorderLaneletTwo =
        std::vector<vertex>{vertex{0, 8},  vertex{10, 8}, vertex{20, 8}, vertex{30, 8}, vertex{40, 8},
                            vertex{50, 8}, vertex{60, 8}, vertex{70, 8}, vertex{80, 8}};
    auto rightBorderLaneletTwo =
        std::vector<vertex>{vertex{0, 4},  vertex{10, 4}, vertex{20, 4}, vertex{30, 4}, vertex{40, 4},
                            vertex{50, 4}, vertex{60, 4}, vertex{70, 4}, vertex{80, 4}};
    auto laneletTwo =
        std::make_shared<Lanelet>(Lanelet(idLaneletTwo, leftBorderLaneletTwo, rightBorderLaneletTwo,
                                          laneletTypeLaneletTwo, userOneWayLaneletTwo, userBidirectionalLaneletTwo));

    return std::make_shared<RoadNetwork>(
        RoadNetwork({laneletOne, laneletTwo}, SupportedTrafficSignCountry::GERMANY, {trafficSignOne, trafficSignTwo}));
}

} // namespace utils_predicate_test