//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/auxiliaryDefs/traffic_signs.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign_element.h>
#include <commonroad_cpp/roadNetwork/road_network.h>

#include "utils_predicate_test.h"

namespace utils_predicate_test {
std::shared_ptr<RoadNetwork> create_road_network(const std::set<LaneletType> &laneletTypeLaneletOne,
                                                 const std::set<LaneletType> &laneletTypeLaneletTwo) {
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
    auto trafficSignTwo{std::make_shared<TrafficSign>(trafficSignIdTwo, trafficSignElementsTwo, vertTwo, false)};
    // stop sign
    size_t trafficSignIdThree = 201;
    std::vector<std::string> trafficSignElementThreeValues;
    auto trafficSignElementsThree{std::vector<std::shared_ptr<TrafficSignElement>>{std::make_shared<TrafficSignElement>(
        trafficSignIDLookupTable->at(TrafficSignTypes::STOP), trafficSignElementThreeValues)}};
    auto vertThree{vertex{40, 0}};
    auto trafficSignThree{
        std::make_shared<TrafficSign>(trafficSignIdThree, trafficSignElementsThree, vertThree, false)};
    // stop line
    std::vector<vertex> slPositionOne{vertex{20.0, 0.0}, vertex{20.0, 4.0}};
    std::vector<vertex> slPositionTwo{vertex{20.0, 4.0}, vertex{21.0, 8.0}};
    auto stopLineOne{std::make_shared<StopLine>(
        slPositionOne, std::vector<std::shared_ptr<TrafficSign>>{trafficSignThree}, LineMarking::broad_solid)};
    auto stopLineTwo{std::make_shared<StopLine>(
        slPositionTwo, std::vector<std::shared_ptr<TrafficSign>>{trafficSignThree}, LineMarking::broad_solid)};
    // right lanelet
    size_t idLaneletOne = 100;
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
    laneletOne->addTrafficSign(trafficSignThree);
    laneletOne->setStopLine(stopLineOne);

    // left lanelet
    size_t idLaneletTwo = 101;
    auto userOneWayLaneletTwo = std::set<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    auto userBidirectionalLaneletTwo = std::set<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    auto leftBorderLaneletTwo = std::vector<vertex>{
        vertex{0, 8},  vertex{10, 8}, vertex{20, 8}, vertex{30, 8},  vertex{40, 8},  vertex{50, 8}, vertex{60, 8},
        vertex{70, 8}, vertex{80, 8}, vertex{90, 8}, vertex{100, 8}, vertex{110, 8}, vertex{120, 8}};
    auto rightBorderLaneletTwo = std::vector<vertex>{
        vertex{0, 4},  vertex{10, 4}, vertex{20, 4}, vertex{30, 4},  vertex{40, 4},  vertex{50, 4}, vertex{60, 4},
        vertex{70, 4}, vertex{80, 4}, vertex{90, 4}, vertex{100, 4}, vertex{110, 4}, vertex{120, 4}};
    auto laneletTwo =
        std::make_shared<Lanelet>(Lanelet(idLaneletTwo, leftBorderLaneletTwo, rightBorderLaneletTwo,
                                          laneletTypeLaneletTwo, userOneWayLaneletTwo, userBidirectionalLaneletTwo));
    laneletTwo->setStopLine(stopLineTwo);

    laneletOne->setLeftAdjacent(laneletTwo, DrivingDirection::same);
    laneletTwo->setRightAdjacent(laneletOne, DrivingDirection::same);

    return std::make_shared<RoadNetwork>(
        RoadNetwork({laneletOne, laneletTwo}, SupportedTrafficSignCountry::GERMANY, {trafficSignOne, trafficSignTwo}));
}

std::shared_ptr<RoadNetwork> create_road_network_2() {
    const auto *trafficSignIDLookupTable = TrafficSignLookupTableByCountry.at(SupportedTrafficSignCountry::GERMANY);
    // max speed traffic sign
    size_t trafficSignIdOne = 200;
    std::vector<std::string> trafficSignElementOneValues{"35"};
    auto trafficSignElementsOne{std::vector<std::shared_ptr<TrafficSignElement>>{std::make_shared<TrafficSignElement>(
        trafficSignIDLookupTable->at(TrafficSignTypes::MAX_SPEED), trafficSignElementOneValues)}};
    auto vertexOne{vertex{5, 0}};
    auto trafficSignOne{std::make_shared<TrafficSign>(trafficSignIdOne, trafficSignElementsOne, vertexOne, false)};
    // required speed traffic sign
    size_t trafficSignIdTwo = 201;
    std::vector<std::string> trafficSignElementTwoValues{"10"};
    auto trafficSignElementsTwo{std::vector<std::shared_ptr<TrafficSignElement>>{std::make_shared<TrafficSignElement>(
        trafficSignIDLookupTable->at(TrafficSignTypes::MIN_SPEED), trafficSignElementTwoValues)}};
    auto vertTwo{vertex{0, 0}};
    auto trafficSignTwo{std::make_shared<TrafficSign>(trafficSignIdTwo, trafficSignElementsTwo, vertTwo, false)};
    // stop sign
    size_t trafficSignIdThree = 201;
    std::vector<std::string> trafficSignElementThreeValues;
    auto trafficSignElementsThree{std::vector<std::shared_ptr<TrafficSignElement>>{std::make_shared<TrafficSignElement>(
        trafficSignIDLookupTable->at(TrafficSignTypes::STOP), trafficSignElementThreeValues)}};
    auto vertThree{vertex{40, 0}};
    auto trafficSignThree{
        std::make_shared<TrafficSign>(trafficSignIdThree, trafficSignElementsThree, vertThree, false)};
    // stop line
    std::vector<vertex> slPositionOne{vertex{20.0, 0.0}, vertex{20.0, 3.0}};
    std::vector<vertex> slPositionTwo{vertex{20.0, 3.0}, vertex{21.0, 6.0}};
    auto stopLineOne{std::make_shared<StopLine>(
        slPositionOne, std::vector<std::shared_ptr<TrafficSign>>{trafficSignThree}, LineMarking::broad_solid)};
    auto stopLineTwo{std::make_shared<StopLine>(
        slPositionTwo, std::vector<std::shared_ptr<TrafficSign>>{trafficSignThree}, LineMarking::broad_solid)};

    // right lanelet
    size_t idLaneletOne = 111;
    auto laneletTypeLaneletOne =
        std::set<LaneletType>{LaneletType::mainCarriageWay, LaneletType::interstate, LaneletType::accessRamp};
    auto userOneWayLaneletOne = std::set<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    auto userBidirectionalLaneletOne = std::set<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    auto leftBorderLaneletOne = std::vector<vertex>{
        vertex{0, 3},   vertex{10, 3},  vertex{20, 3},  vertex{30, 3},  vertex{40, 3},  vertex{50, 3},
        vertex{60, 3},  vertex{70, 3},  vertex{80, 3},  vertex{90, 3},  vertex{100, 3}, vertex{110, 3},
        vertex{120, 3}, vertex{130, 3}, vertex{140, 3}, vertex{150, 3}, vertex{160, 3}, vertex{170, 3}};
    auto rightBorderLaneletOne = std::vector<vertex>{
        vertex{0, 0},   vertex{10, 0},  vertex{20, 0},  vertex{30, 0},  vertex{40, 0},  vertex{50, 0},
        vertex{60, 0},  vertex{70, 0},  vertex{80, 0},  vertex{90, 0},  vertex{100, 0}, vertex{110, 0},
        vertex{120, 0}, vertex{130, 0}, vertex{140, 0}, vertex{150, 0}, vertex{160, 0}, vertex{170, 0}};
    auto laneletOne2 =
        std::make_shared<Lanelet>(Lanelet(idLaneletOne, leftBorderLaneletOne, rightBorderLaneletOne,
                                          laneletTypeLaneletOne, userOneWayLaneletOne, userBidirectionalLaneletOne));
    laneletOne2->addTrafficSign(trafficSignOne);
    laneletOne2->addTrafficSign(trafficSignTwo);
    laneletOne2->addTrafficSign(trafficSignThree);
    laneletOne2->setStopLine(stopLineOne);

    // left lanelet
    size_t idLaneletTwo = 222;
    auto laneletTypeLaneletTwo = std::set<LaneletType>{LaneletType::mainCarriageWay, LaneletType::interstate};
    auto userOneWayLaneletTwo = std::set<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    auto userBidirectionalLaneletTwo = std::set<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    auto leftBorderLaneletTwo =
        std::vector<vertex>{vertex{0, 6},  vertex{10, 6}, vertex{20, 6}, vertex{30, 6}, vertex{40, 6},
                            vertex{50, 6}, vertex{60, 6}, vertex{70, 6}, vertex{80, 6}};
    auto rightBorderLaneletTwo =
        std::vector<vertex>{vertex{0, 3},  vertex{10, 3}, vertex{20, 3}, vertex{30, 3}, vertex{40, 3},
                            vertex{50, 3}, vertex{60, 3}, vertex{70, 3}, vertex{80, 3}};
    auto laneletTwo2 =
        std::make_shared<Lanelet>(Lanelet(idLaneletTwo, leftBorderLaneletTwo, rightBorderLaneletTwo,
                                          laneletTypeLaneletTwo, userOneWayLaneletTwo, userBidirectionalLaneletTwo));
    laneletTwo2->setStopLine(stopLineTwo);

    laneletOne2->setLeftAdjacent(laneletTwo2, DrivingDirection::same);
    laneletTwo2->setRightAdjacent(laneletOne2, DrivingDirection::same);

    // third lanelet
    size_t idLaneletThree = 333;
    auto laneletTypeLaneletThree = std::set<LaneletType>{LaneletType::shoulder, LaneletType::interstate};
    auto userOneWayLaneletThree = std::set<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    auto userBidirectionalLaneletThree = std::set<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    auto rightBorderLaneletThree =
        std::vector<vertex>{vertex{0, 6},  vertex{10, 6}, vertex{20, 6}, vertex{30, 6}, vertex{40, 6},
                            vertex{50, 6}, vertex{60, 6}, vertex{70, 6}, vertex{80, 6}};
    auto leftBorderLaneletThree =
        std::vector<vertex>{vertex{0, 8},  vertex{10, 8}, vertex{20, 8}, vertex{30, 8}, vertex{40, 8},
                            vertex{50, 8}, vertex{60, 8}, vertex{70, 8}, vertex{80, 8}};
    auto laneletThree2 = std::make_shared<Lanelet>(Lanelet(idLaneletThree, leftBorderLaneletThree,
                                                           rightBorderLaneletThree, laneletTypeLaneletThree,
                                                           userOneWayLaneletThree, userBidirectionalLaneletThree));

    laneletThree2->setRightAdjacent(laneletTwo2, DrivingDirection::same);
    laneletTwo2->setLeftAdjacent(laneletThree2, DrivingDirection::same);

    // fourth lanelet
    size_t idLaneletFour = 444;
    auto laneletTypeLaneletFour = std::set<LaneletType>{LaneletType::exitRamp, LaneletType::interstate};
    auto rightBorderLaneletFour = std::vector<vertex>{vertex{80, 6}, vertex{90, 6}, vertex{100, 6}};
    auto leftBorderLaneletFour = std::vector<vertex>{vertex{80, 8}, vertex{90, 8}, vertex{100, 8}};
    auto laneletFour2 = std::make_shared<Lanelet>(
        Lanelet(idLaneletFour, leftBorderLaneletFour, rightBorderLaneletFour, laneletTypeLaneletThree));

    laneletFour2->addPredecessor(laneletTwo2);
    laneletTwo2->addSuccessor(laneletFour2);

    return std::make_shared<RoadNetwork>(RoadNetwork({laneletOne2, laneletTwo2, laneletThree2, laneletFour2},
                                                     SupportedTrafficSignCountry::GERMANY,
                                                     {trafficSignOne, trafficSignTwo}));
}

std::shared_ptr<RoadNetwork> create_road_network_3() {
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
    auto trafficSignTwo{std::make_shared<TrafficSign>(trafficSignIdTwo, trafficSignElementsTwo, vertTwo, false)};
    // stop sign
    size_t trafficSignIdThree = 201;
    std::vector<std::string> trafficSignElementThreeValues;
    auto trafficSignElementsThree{std::vector<std::shared_ptr<TrafficSignElement>>{std::make_shared<TrafficSignElement>(
        trafficSignIDLookupTable->at(TrafficSignTypes::STOP), trafficSignElementThreeValues)}};
    auto vertThree{vertex{40, 0}};
    auto trafficSignThree{
        std::make_shared<TrafficSign>(trafficSignIdThree, trafficSignElementsThree, vertThree, false)};
    // stop line
    std::vector<vertex> slPositionOne{vertex{20.0, 0.0}, vertex{20.0, 3.0}};
    std::vector<vertex> slPositionTwo{vertex{20.0, 3.0}, vertex{21.0, 6.0}};
    auto stopLineOne{std::make_shared<StopLine>(
        slPositionOne, std::vector<std::shared_ptr<TrafficSign>>{trafficSignThree}, LineMarking::broad_solid)};
    auto stopLineTwo{std::make_shared<StopLine>(
        slPositionTwo, std::vector<std::shared_ptr<TrafficSign>>{trafficSignThree}, LineMarking::broad_solid)};

    // lanelet 1
    size_t idLaneletOne = 100;
    auto laneletTypeLaneletOne =
        std::set<LaneletType>{LaneletType::mainCarriageWay, LaneletType::interstate, LaneletType::accessRamp};
    auto userOneWayLaneletOne = std::set<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    auto userBidirectionalLaneletOne = std::set<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    auto leftBorderLaneletOne =
        std::vector<vertex>{vertex{0, 4},  vertex{10, 4}, vertex{20, 4}, vertex{30, 4}, vertex{40, 4},
                            vertex{50, 4}, vertex{60, 4}, vertex{70, 4}, vertex{80, 4}};
    auto rightBorderLaneletOne =
        std::vector<vertex>{vertex{0, 0},  vertex{10, 0}, vertex{20, 0}, vertex{30, 0}, vertex{40, 0},
                            vertex{50, 0}, vertex{60, 0}, vertex{70, 0}, vertex{80, 0}};
    auto laneletOne =
        std::make_shared<Lanelet>(Lanelet(idLaneletOne, leftBorderLaneletOne, rightBorderLaneletOne,
                                          laneletTypeLaneletOne, userOneWayLaneletOne, userBidirectionalLaneletOne));
    laneletOne->addTrafficSign(trafficSignOne);
    laneletOne->addTrafficSign(trafficSignTwo);
    laneletOne->addTrafficSign(trafficSignThree);
    laneletOne->setStopLine(stopLineOne);

    // lanelet2
    size_t idLaneletTwo = 200;
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
    laneletTwo->setStopLine(stopLineTwo);

    laneletOne->setLeftAdjacent(laneletTwo, DrivingDirection::same);
    laneletTwo->setRightAdjacent(laneletOne, DrivingDirection::same);
    laneletTwo->setLineMarkingLeft(LineMarking::broad_dashed);

    // third lanelet
    size_t idLaneletThree = 300;
    auto laneletTypeLaneletThree = std::set<LaneletType>{LaneletType::shoulder, LaneletType::interstate};
    auto userOneWayLaneletThree = std::set<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    auto userBidirectionalLaneletThree = std::set<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    auto rightBorderLaneletThree =
        std::vector<vertex>{vertex{0, 8},  vertex{10, 8}, vertex{20, 8}, vertex{30, 8}, vertex{40, 8},
                            vertex{50, 8}, vertex{60, 8}, vertex{70, 8}, vertex{80, 8}};
    auto leftBorderLaneletThree =
        std::vector<vertex>{vertex{0, 12},  vertex{10, 12}, vertex{20, 12}, vertex{30, 12}, vertex{40, 12},
                            vertex{50, 12}, vertex{60, 12}, vertex{70, 12}, vertex{80, 12}};
    auto laneletThree = std::make_shared<Lanelet>(Lanelet(idLaneletThree, leftBorderLaneletThree,
                                                          rightBorderLaneletThree, laneletTypeLaneletThree,
                                                          userOneWayLaneletThree, userBidirectionalLaneletThree));

    laneletThree->setRightAdjacent(laneletTwo, DrivingDirection::same);
    laneletTwo->setLeftAdjacent(laneletThree, DrivingDirection::same);
    laneletThree->setLineMarkingLeft(LineMarking::broad_dashed);
    laneletThree->setLineMarkingRight(LineMarking::broad_dashed);

    // fourth lanelet
    size_t idLaneletFour = 400;
    auto laneletTypeLaneletFour = std::set<LaneletType>{LaneletType::shoulder, LaneletType::interstate};
    auto userOneWayLaneletFour = std::set<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    auto userBidirectionalLaneletFour = std::set<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    auto rightBorderLaneletFour =
        std::vector<vertex>{vertex{0, 12},  vertex{10, 12}, vertex{20, 12}, vertex{30, 12}, vertex{40, 12},
                            vertex{50, 12}, vertex{60, 12}, vertex{70, 12}, vertex{80, 12}};
    auto leftBorderLaneletFour =
        std::vector<vertex>{vertex{0, 16},  vertex{10, 16}, vertex{20, 16}, vertex{30, 16}, vertex{40, 16},
                            vertex{50, 16}, vertex{60, 16}, vertex{70, 16}, vertex{80, 16}};
    auto laneletFour =
        std::make_shared<Lanelet>(Lanelet(idLaneletFour, leftBorderLaneletFour, rightBorderLaneletFour,
                                          laneletTypeLaneletFour, userOneWayLaneletFour, userBidirectionalLaneletFour));

    laneletFour->setRightAdjacent(laneletThree, DrivingDirection::same);
    laneletThree->setLeftAdjacent(laneletFour, DrivingDirection::same);
    laneletFour->setLineMarkingLeft(LineMarking::broad_dashed);
    laneletFour->setLineMarkingRight(LineMarking::broad_dashed);

    // fifth lanelet
    size_t idLaneletFive = 500;
    auto laneletTypeLaneletFive = std::set<LaneletType>{LaneletType::shoulder, LaneletType::interstate};
    auto userOneWayLaneletFive = std::set<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    auto userBidirectionalLaneletFive = std::set<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    auto rightBorderLaneletFive =
        std::vector<vertex>{vertex{0, 16},  vertex{10, 16}, vertex{20, 16}, vertex{30, 16}, vertex{40, 16},
                            vertex{50, 16}, vertex{60, 16}, vertex{70, 16}, vertex{80, 16}};
    auto leftBorderLaneletFive =
        std::vector<vertex>{vertex{0, 20},  vertex{10, 20}, vertex{20, 20}, vertex{30, 20}, vertex{40, 20},
                            vertex{50, 20}, vertex{60, 20}, vertex{70, 20}, vertex{80, 20}};
    auto laneletFive =
        std::make_shared<Lanelet>(Lanelet(idLaneletFive, leftBorderLaneletFive, rightBorderLaneletFive,
                                          laneletTypeLaneletFive, userOneWayLaneletFive, userBidirectionalLaneletFive));

    laneletFive->setRightAdjacent(laneletFour, DrivingDirection::same);
    laneletFour->setLeftAdjacent(laneletFive, DrivingDirection::same);
    laneletFive->setLineMarkingRight(LineMarking::broad_dashed);

    // sixth lanelet
    size_t idLaneletSix = 600;
    auto laneletTypeLaneletSix = std::set<LaneletType>{LaneletType::shoulder, LaneletType::interstate};
    auto userOneWayLaneletSix = std::set<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    auto userBidirectionalLaneletSix = std::set<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    auto rightBorderLaneletSix =
        std::vector<vertex>{vertex{0, 20},  vertex{10, 20}, vertex{20, 20}, vertex{30, 20}, vertex{40, 20},
                            vertex{50, 20}, vertex{60, 20}, vertex{70, 20}, vertex{80, 20}};
    auto leftBorderLaneletSix =
        std::vector<vertex>{vertex{0, 24},  vertex{10, 24}, vertex{20, 24}, vertex{30, 24}, vertex{40, 24},
                            vertex{50, 24}, vertex{60, 24}, vertex{70, 24}, vertex{80, 24}};
    auto laneletSix =
        std::make_shared<Lanelet>(Lanelet(idLaneletSix, leftBorderLaneletSix, rightBorderLaneletSix,
                                          laneletTypeLaneletSix, userOneWayLaneletSix, userBidirectionalLaneletSix));

    laneletSix->setRightAdjacent(laneletFive, DrivingDirection::same);
    laneletFive->setLeftAdjacent(laneletSix, DrivingDirection::same);

    return std::make_shared<RoadNetwork>(
        RoadNetwork({laneletOne, laneletTwo, laneletThree, laneletFour, laneletFive, laneletSix},
                    SupportedTrafficSignCountry::GERMANY, {trafficSignOne, trafficSignTwo}));
}
} // namespace utils_predicate_test
