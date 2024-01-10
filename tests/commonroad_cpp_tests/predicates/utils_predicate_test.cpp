#include <commonroad_cpp/auxiliaryDefs/regulatory_elements.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign_element.h>
#include <commonroad_cpp/roadNetwork/road_network.h>

#include "utils_predicate_test.h"

namespace utils_predicate_test {
std::shared_ptr<RoadNetwork> create_road_network(const std::set<LaneletType> &laneletTypeLaneletOne,
                                                 const std::set<LaneletType> &laneletTypeLaneletTwo) {
    // max speed traffic sign
    size_t trafficSignIdOne = 200;
    std::vector<std::string> trafficSignElementOneValues{"50"};
    auto trafficSignElementsOne{std::vector<std::shared_ptr<TrafficSignElement>>{
        std::make_shared<TrafficSignElement>(TrafficSignTypes::MAX_SPEED, trafficSignElementOneValues)}};
    auto vertexOne{vertex{0, 0}};
    auto trafficSignOne{std::make_shared<TrafficSign>(trafficSignIdOne, trafficSignElementsOne, vertexOne, false)};
    // required speed traffic sign
    size_t trafficSignIdTwo = 201;
    std::vector<std::string> trafficSignElementTwoValues{"10"};
    auto trafficSignElementsTwo{std::vector<std::shared_ptr<TrafficSignElement>>{
        std::make_shared<TrafficSignElement>(TrafficSignTypes::MIN_SPEED, trafficSignElementTwoValues)}};
    auto vertTwo{vertex{0, 0}};
    auto trafficSignTwo{std::make_shared<TrafficSign>(trafficSignIdTwo, trafficSignElementsTwo, vertTwo, false)};
    // stop sign
    size_t trafficSignIdThree = 201;
    std::vector<std::string> trafficSignElementThreeValues;
    auto trafficSignElementsThree{std::vector<std::shared_ptr<TrafficSignElement>>{
        std::make_shared<TrafficSignElement>(TrafficSignTypes::STOP, trafficSignElementThreeValues)}};
    auto vertThree{vertex{40, 0}};
    auto trafficSignThree{
        std::make_shared<TrafficSign>(trafficSignIdThree, trafficSignElementsThree, vertThree, false)};
    // stop line
    std::pair<vertex, vertex> slPositionOne{{20.0, 0.0}, {20.0, 4.0}};
    std::pair<vertex, vertex> slPositionTwo{{20.0, 4.0}, {21.0, 8.0}};
    auto stopLineOne{std::make_shared<StopLine>(slPositionOne, LineMarking::broad_solid)};
    auto stopLineTwo{std::make_shared<StopLine>(slPositionTwo, LineMarking::broad_solid)};
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

    laneletOne->setLeftAdjacent(laneletTwo, false);
    laneletTwo->setRightAdjacent(laneletOne, false);

    return std::make_shared<RoadNetwork>(
        RoadNetwork({laneletOne, laneletTwo}, SupportedTrafficSignCountry::GERMANY, {trafficSignOne, trafficSignTwo}));
}

std::shared_ptr<RoadNetwork> create_road_network_2() {
    // max speed traffic sign
    size_t trafficSignIdOne = 200;
    std::vector<std::string> trafficSignElementOneValues{"35"};
    auto trafficSignElementsOne{std::vector<std::shared_ptr<TrafficSignElement>>{
        std::make_shared<TrafficSignElement>(TrafficSignTypes::MAX_SPEED, trafficSignElementOneValues)}};
    auto vertexOne{vertex{5, 0}};
    auto trafficSignOne{std::make_shared<TrafficSign>(trafficSignIdOne, trafficSignElementsOne, vertexOne, false)};
    // required speed traffic sign
    size_t trafficSignIdTwo = 201;
    std::vector<std::string> trafficSignElementTwoValues{"10"};
    auto trafficSignElementsTwo{std::vector<std::shared_ptr<TrafficSignElement>>{
        std::make_shared<TrafficSignElement>(TrafficSignTypes::MIN_SPEED, trafficSignElementTwoValues)}};
    auto vertTwo{vertex{0, 0}};
    auto trafficSignTwo{std::make_shared<TrafficSign>(trafficSignIdTwo, trafficSignElementsTwo, vertTwo, false)};
    // stop sign
    size_t trafficSignIdThree = 201;
    std::vector<std::string> trafficSignElementThreeValues;
    auto trafficSignElementsThree{std::vector<std::shared_ptr<TrafficSignElement>>{
        std::make_shared<TrafficSignElement>(TrafficSignTypes::STOP, trafficSignElementThreeValues)}};
    auto vertThree{vertex{40, 0}};
    auto trafficSignThree{
        std::make_shared<TrafficSign>(trafficSignIdThree, trafficSignElementsThree, vertThree, false)};
    // stop line
    std::pair<vertex, vertex> slPositionOne{{20.0, 0.0}, {20.0, 3.0}};
    std::pair<vertex, vertex> slPositionTwo{{20.0, 3.0}, {21.0, 6.0}};
    auto stopLineOne{std::make_shared<StopLine>(slPositionOne, LineMarking::broad_solid)};
    auto stopLineTwo{std::make_shared<StopLine>(slPositionTwo, LineMarking::broad_solid)};

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
    auto laneletTypeLaneletTwo =
        std::set<LaneletType>{LaneletType::mainCarriageWay, LaneletType::interstate, LaneletType::incoming};
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

    laneletOne2->setLeftAdjacent(laneletTwo2, false);
    laneletTwo2->setRightAdjacent(laneletOne2, false);

    // third lanelet
    size_t idLaneletThree = 333;
    auto laneletTypeLaneletThree =
        std::set<LaneletType>{LaneletType::shoulder, LaneletType::interstate, LaneletType::incoming};
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

    laneletThree2->setRightAdjacent(laneletTwo2, false);
    laneletTwo2->setLeftAdjacent(laneletThree2, false);

    // fourth lanelet
    size_t idLaneletFour = 444;
    auto laneletTypeLaneletFour = std::set<LaneletType>{LaneletType::exitRamp, LaneletType::interstate};
    auto rightBorderLaneletFour = std::vector<vertex>{vertex{80, 6}, vertex{90, 6}, vertex{100, 6}};
    auto leftBorderLaneletFour = std::vector<vertex>{vertex{80, 8}, vertex{90, 8}, vertex{100, 8}};
    auto laneletFour2 = std::make_shared<Lanelet>(
        Lanelet(idLaneletFour, leftBorderLaneletFour, rightBorderLaneletFour, laneletTypeLaneletThree));

    laneletFour2->addPredecessor(laneletThree2);
    laneletThree2->addSuccessor(laneletFour2);

    return std::make_shared<RoadNetwork>(RoadNetwork({laneletOne2, laneletTwo2, laneletThree2, laneletFour2},
                                                     SupportedTrafficSignCountry::GERMANY,
                                                     {trafficSignOne, trafficSignTwo}));
}

std::shared_ptr<RoadNetwork> create_road_network_3() {
    // max speed traffic sign
    size_t trafficSignIdOne = 200;
    std::vector<std::string> trafficSignElementOneValues{"50"};
    auto trafficSignElementsOne{std::vector<std::shared_ptr<TrafficSignElement>>{
        std::make_shared<TrafficSignElement>(TrafficSignTypes::MAX_SPEED, trafficSignElementOneValues)}};
    auto vertexOne{vertex{0, 0}};
    auto trafficSignOne{std::make_shared<TrafficSign>(trafficSignIdOne, trafficSignElementsOne, vertexOne, false)};
    // required speed traffic sign
    size_t trafficSignIdTwo = 201;
    std::vector<std::string> trafficSignElementTwoValues{"10"};
    auto trafficSignElementsTwo{std::vector<std::shared_ptr<TrafficSignElement>>{
        std::make_shared<TrafficSignElement>(TrafficSignTypes::MIN_SPEED, trafficSignElementTwoValues)}};
    auto vertTwo{vertex{0, 0}};
    auto trafficSignTwo{std::make_shared<TrafficSign>(trafficSignIdTwo, trafficSignElementsTwo, vertTwo, false)};
    // stop sign
    size_t trafficSignIdThree = 201;
    std::vector<std::string> trafficSignElementThreeValues;
    auto trafficSignElementsThree{std::vector<std::shared_ptr<TrafficSignElement>>{
        std::make_shared<TrafficSignElement>(TrafficSignTypes::STOP, trafficSignElementThreeValues)}};
    auto vertThree{vertex{40, 0}};
    auto trafficSignThree{
        std::make_shared<TrafficSign>(trafficSignIdThree, trafficSignElementsThree, vertThree, false)};
    // stop line
    std::pair<vertex, vertex> slPositionOne{{20.0, 0.0}, {20.0, 3.0}};
    std::pair<vertex, vertex> slPositionTwo{{20.0, 3.0}, {21.0, 6.0}};
    auto stopLineOne{std::make_shared<StopLine>(slPositionOne, LineMarking::broad_solid)};
    auto stopLineTwo{std::make_shared<StopLine>(slPositionTwo, LineMarking::broad_solid)};

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

    laneletOne->setLeftAdjacent(laneletTwo, false);
    laneletTwo->setRightAdjacent(laneletOne, false);
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

    laneletThree->setRightAdjacent(laneletTwo, false);
    laneletTwo->setLeftAdjacent(laneletThree, false);
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

    laneletFour->setRightAdjacent(laneletThree, false);
    laneletThree->setLeftAdjacent(laneletFour, false);
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

    laneletFive->setRightAdjacent(laneletFour, false);
    laneletFour->setLeftAdjacent(laneletFive, false);
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

    laneletSix->setRightAdjacent(laneletFive, false);
    laneletFive->setLeftAdjacent(laneletSix, false);

    return std::make_shared<RoadNetwork>(
        RoadNetwork({laneletOne, laneletTwo, laneletThree, laneletFour, laneletFive, laneletSix},
                    SupportedTrafficSignCountry::GERMANY, {trafficSignOne, trafficSignTwo}));
}

/**
 * Creates a roadnetwork as follows (l = left, r = right, sl = successor left, sr = successor right):
 *          l l l l l l l l l l sl sl sl sl sl sl sl sl sl sl
 *          r r r r r r r r r r sr sr sr sr sr sr sr sr sr sr
 * @param laneletTypeRight : r
 * @param laneletTypeLeft : l
 * @param laneletTypeSuccessorRight : sr
 * @param laneletTypeSuccessorLeft : sl
 * @return a roadnetwork as shown above to test different successor types
 */
std::shared_ptr<RoadNetwork> create_road_network_with_2_successors(
    const std::set<LaneletType> &laneletTypeRight, const std::set<LaneletType> &laneletTypeLeft,
    const std::set<LaneletType> &laneletTypeSuccessorRight, const std::set<LaneletType> &laneletTypeSuccessorLeft) {
    // right lanelet
    size_t idLaneletRight = 100;
    auto userOneWayLaneletRight = std::set<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    auto userBidirectionalLaneletRight = std::set<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    auto leftBorderLaneletRight =
        std::vector<vertex>{vertex{0, 4},  vertex{10, 4}, vertex{20, 4}, vertex{30, 4}, vertex{40, 4}, vertex{50, 4},
                            vertex{60, 4}, vertex{70, 4}, vertex{80, 4}, vertex{90, 4}, vertex{100, 4}};
    auto rightBorderLaneletRight =
        std::vector<vertex>{vertex{0, 0},  vertex{10, 0}, vertex{20, 0}, vertex{30, 0}, vertex{40, 0}, vertex{50, 0},
                            vertex{60, 0}, vertex{70, 0}, vertex{80, 0}, vertex{90, 0}, vertex{100, 0}};
    auto laneletRight =
        std::make_shared<Lanelet>(Lanelet(idLaneletRight, leftBorderLaneletRight, rightBorderLaneletRight,
                                          laneletTypeRight, userOneWayLaneletRight, userBidirectionalLaneletRight));

    // right successor
    size_t idLaneletSuccessorRight = 200;
    auto userOneWayLaneletSuccessorRight = std::set<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    auto userBidirectionalLaneletSuccessorRight = std::set<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    auto leftBorderLaneletSuccessorRight = std::vector<vertex>{
        vertex{100, 4}, vertex{110, 4}, vertex{120, 4}, vertex{130, 4}, vertex{140, 4}, vertex{150, 4},
        vertex{160, 4}, vertex{170, 4}, vertex{180, 4}, vertex{190, 4}, vertex{200, 4}};
    auto rightBorderLaneletSuccessorRight = std::vector<vertex>{
        vertex{100, 0}, vertex{110, 0}, vertex{120, 0}, vertex{130, 0}, vertex{140, 0}, vertex{150, 0},
        vertex{160, 0}, vertex{170, 0}, vertex{180, 0}, vertex{190, 0}, vertex{200, 0}};
    auto laneletSuccessorRight = std::make_shared<Lanelet>(
        Lanelet(idLaneletSuccessorRight, leftBorderLaneletSuccessorRight, rightBorderLaneletSuccessorRight,
                laneletTypeSuccessorRight, userOneWayLaneletSuccessorRight, userBidirectionalLaneletSuccessorRight));

    // left lanelet
    size_t idLaneletLeft = 101;
    auto userOneWayLaneletLeft = std::set<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    auto userBidirectionalLaneletLeft = std::set<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    auto leftBorderLaneletLeft =
        std::vector<vertex>{vertex{0, 8},  vertex{10, 8}, vertex{20, 8}, vertex{30, 8}, vertex{40, 8}, vertex{50, 8},
                            vertex{60, 8}, vertex{70, 8}, vertex{80, 8}, vertex{90, 8}, vertex{100, 8}};
    auto rightBorderLaneletLeft =
        std::vector<vertex>{vertex{0, 4},  vertex{10, 4}, vertex{20, 4}, vertex{30, 4}, vertex{40, 4}, vertex{50, 4},
                            vertex{60, 4}, vertex{70, 4}, vertex{80, 4}, vertex{90, 4}, vertex{100, 4}};
    auto laneletLeft =
        std::make_shared<Lanelet>(Lanelet(idLaneletLeft, leftBorderLaneletLeft, rightBorderLaneletLeft, laneletTypeLeft,
                                          userOneWayLaneletLeft, userBidirectionalLaneletLeft));

    // left successor
    size_t idLaneletSuccessorLeft = 201;
    auto userOneWayLaneletSuccessorLeft = std::set<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    auto userBidirectionalLaneletSuccessorLeft = std::set<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    auto leftBorderLaneletSuccessorLeft = std::vector<vertex>{
        vertex{100, 8}, vertex{110, 8}, vertex{120, 8}, vertex{130, 8}, vertex{140, 8}, vertex{150, 8},
        vertex{160, 8}, vertex{170, 8}, vertex{180, 8}, vertex{190, 8}, vertex{200, 8}};
    auto rightBorderLaneletSuccessorLeft = std::vector<vertex>{
        vertex{100, 4}, vertex{110, 4}, vertex{120, 4}, vertex{130, 4}, vertex{140, 4}, vertex{150, 4},
        vertex{160, 4}, vertex{170, 4}, vertex{180, 4}, vertex{190, 4}, vertex{200, 4}};
    auto laneletSuccessorLeft = std::make_shared<Lanelet>(
        Lanelet(idLaneletSuccessorLeft, leftBorderLaneletSuccessorLeft, rightBorderLaneletSuccessorLeft,
                laneletTypeSuccessorLeft, userOneWayLaneletSuccessorLeft, userBidirectionalLaneletSuccessorLeft));

    // Dependencies between the Lanelets
    laneletLeft->setRightAdjacent(laneletRight, false);
    laneletLeft->addSuccessor(laneletSuccessorLeft);
    laneletRight->setLeftAdjacent(laneletLeft, false);
    laneletRight->addSuccessor(laneletSuccessorRight);
    laneletSuccessorLeft->setRightAdjacent(laneletSuccessorRight, false);
    laneletSuccessorLeft->addPredecessor(laneletLeft);
    laneletSuccessorRight->setLeftAdjacent(laneletSuccessorLeft, false);
    laneletSuccessorRight->addPredecessor(laneletRight);

    return std::make_shared<RoadNetwork>(
        RoadNetwork({laneletLeft, laneletRight, laneletSuccessorLeft, laneletSuccessorRight}));
}

/**
 * Creates a roadnetwork as follows (u = unknown, sr = successor right, sl = successor left):
 *          t t t t t t
 *        l             r
 *        l             r
 *        l             r
 *        l             r
 *        l             r
 *        l             r
 *          b b b b b b
 * @param laneletTypeRight : r
 * @param laneletTypeLeft : l
 * @param laneletTypeBottom : b
 * @param laneletTypeTop : t
 * @return a roadnetwork as shown above to test different successor types
 */
std::shared_ptr<RoadNetwork> create_road_network_with_circle(const std::set<LaneletType> &laneletTypeBottom,
                                                             const std::set<LaneletType> &laneletTypeLeft,
                                                             const std::set<LaneletType> &laneletTypeRight,
                                                             const std::set<LaneletType> &laneletTypeTop) {
    // bottom lanelet
    size_t idLaneletBottom = 100;
    auto userOneWayLaneletBottom = std::set<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    auto userBidirectionalLaneletBottom = std::set<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    auto leftBorderLaneletBottom =
        std::vector<vertex>{vertex{4, 4}, vertex{10, 4}, vertex{20, 4}, vertex{30, 4}, vertex{40, 4}, vertex{46, 4}};
    auto rightBorderLaneletBottom =
        std::vector<vertex>{vertex{0, 0}, vertex{10, 0}, vertex{20, 0}, vertex{30, 0}, vertex{40, 0}, vertex{50, 0}};
    auto laneletBottom =
        std::make_shared<Lanelet>(Lanelet(idLaneletBottom, leftBorderLaneletBottom, rightBorderLaneletBottom,
                                          laneletTypeBottom, userOneWayLaneletBottom, userBidirectionalLaneletBottom));

    // right lanelet
    size_t idLaneletRight = 101;
    auto userOneWayLaneletRight = std::set<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    auto userBidirectionalLaneletRight = std::set<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    auto leftBorderLaneletRight = std::vector<vertex>{vertex{46, 4},  vertex{46, 10}, vertex{46, 20},
                                                      vertex{46, 30}, vertex{46, 40}, vertex{46, 46}};
    auto rightBorderLaneletRight = std::vector<vertex>{vertex{50, 0},  vertex{50, 10}, vertex{50, 20},
                                                       vertex{50, 30}, vertex{50, 40}, vertex{50, 50}};
    auto laneletRight =
        std::make_shared<Lanelet>(Lanelet(idLaneletRight, leftBorderLaneletRight, rightBorderLaneletRight,
                                          laneletTypeRight, userOneWayLaneletRight, userBidirectionalLaneletRight));

    // left lanelet
    size_t idLaneletLeft = 102;
    auto userOneWayLaneletLeft = std::set<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    auto userBidirectionalLaneletLeft = std::set<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    auto leftBorderLaneletLeft =
        std::vector<vertex>{vertex{0, 50}, vertex{0, 40}, vertex{0, 30}, vertex{0, 20}, vertex{0, 10}, vertex{0, 0}};
    auto rightBorderLaneletLeft =
        std::vector<vertex>{vertex{4, 46}, vertex{4, 40}, vertex{4, 30}, vertex{4, 20}, vertex{4, 10}, vertex{4, 4}};
    auto laneletLeft =
        std::make_shared<Lanelet>(Lanelet(idLaneletLeft, leftBorderLaneletLeft, rightBorderLaneletLeft, laneletTypeLeft,
                                          userOneWayLaneletLeft, userBidirectionalLaneletLeft));

    // top lanelet
    size_t idLaneletTop = 103;
    auto userOneWayLaneletTop = std::set<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    auto userBidirectionalLaneletTop = std::set<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    auto leftBorderLaneletTop = std::vector<vertex>{vertex{46, 46}, vertex{40, 46}, vertex{30, 46},
                                                    vertex{20, 46}, vertex{10, 46}, vertex{4, 46}};
    auto rightBorderLaneletTop = std::vector<vertex>{vertex{50, 50}, vertex{40, 50}, vertex{30, 50},
                                                     vertex{20, 50}, vertex{10, 50}, vertex{0, 50}};
    auto laneletTop =
        std::make_shared<Lanelet>(Lanelet(idLaneletTop, leftBorderLaneletTop, rightBorderLaneletTop, laneletTypeTop,
                                          userOneWayLaneletTop, userBidirectionalLaneletTop));

    // lanelet ongoing
    size_t idLaneletOngoing = 200;
    auto userOneWayLaneletOngoing = std::set<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    auto userBidirectionalLaneletOngoing = std::set<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    auto leftBorderLaneletOngoing = std::vector<vertex>{vertex{46, 46}, vertex{40, 46}, vertex{30, 46},
                                                        vertex{20, 46}, vertex{10, 46}, vertex{4, 46}};
    auto rightBorderLaneletOngoing = std::vector<vertex>{vertex{50, 50}, vertex{40, 50}, vertex{30, 50},
                                                         vertex{20, 50}, vertex{10, 50}, vertex{0, 50}};
    auto laneletOngoing = std::make_shared<Lanelet>(Lanelet(idLaneletOngoing, leftBorderLaneletTop,
                                                            rightBorderLaneletTop, {LaneletType::incoming},
                                                            userOneWayLaneletOngoing, userBidirectionalLaneletTop));

    // Dependencies between the Lanelets (circle)
    laneletLeft->addSuccessor(laneletBottom);
    laneletBottom->addSuccessor(laneletRight);
    laneletBottom->addSuccessor(laneletOngoing);
    laneletRight->addSuccessor(laneletTop);
    laneletTop->addSuccessor(laneletLeft);

    return std::make_shared<RoadNetwork>(
        RoadNetwork({laneletLeft, laneletRight, laneletTop, laneletBottom, laneletOngoing}));
}

std::shared_ptr<RoadNetwork> create_narrow_road_network(double width) {
    size_t idLaneletOne = 100;
    auto userBidirectionalLaneletOne = std::set<ObstacleType>{ObstacleType::car};
    auto leftBorderLaneletOne = std::vector<vertex>{
        vertex{0, width},   vertex{10, width},  vertex{20, width},  vertex{30, width},  vertex{40, width},
        vertex{50, width},  vertex{60, width},  vertex{70, width},  vertex{80, width},  vertex{90, width},
        vertex{100, width}, vertex{110, width}, vertex{120, width}, vertex{130, width}, vertex{140, width},
        vertex{150, width}, vertex{160, width}, vertex{170, width}};
    auto rightBorderLaneletOne = std::vector<vertex>{
        vertex{0, 0},   vertex{10, 0},  vertex{20, 0},  vertex{30, 0},  vertex{40, 0},  vertex{50, 0},
        vertex{60, 0},  vertex{70, 0},  vertex{80, 0},  vertex{90, 0},  vertex{100, 0}, vertex{110, 0},
        vertex{120, 0}, vertex{130, 0}, vertex{140, 0}, vertex{150, 0}, vertex{160, 0}, vertex{170, 0}};

    auto laneletOne =
        std::make_shared<Lanelet>(Lanelet(idLaneletOne, leftBorderLaneletOne, rightBorderLaneletOne,
                                          {LaneletType::mainCarriageWay}, {}, userBidirectionalLaneletOne));

    return std::make_shared<RoadNetwork>(RoadNetwork({laneletOne}, SupportedTrafficSignCountry::GERMANY));
}

std::shared_ptr<RoadNetwork> create_road_network_users(const std::set<ObstacleType> &userOneWayLanelet,
                                                       const std::set<ObstacleType> &userBidirectionalLanelet) {
    size_t idLanelet = 100;
    auto leftBorderLanelet = std::vector<vertex>{
        vertex{0, 3.5},   vertex{10, 3.5},  vertex{20, 3.5},  vertex{30, 3.5},  vertex{40, 3.5},  vertex{50, 3.5},
        vertex{60, 3.5},  vertex{70, 3.5},  vertex{80, 3.5},  vertex{90, 3.5},  vertex{100, 3.5}, vertex{110, 3.5},
        vertex{120, 3.5}, vertex{130, 3.5}, vertex{140, 3.5}, vertex{150, 3.5}, vertex{160, 3.5}, vertex{170, 3.5}};
    auto rightBorderLanelet = std::vector<vertex>{
        vertex{0, 0},   vertex{10, 0},  vertex{20, 0},  vertex{30, 0},  vertex{40, 0},  vertex{50, 0},
        vertex{60, 0},  vertex{70, 0},  vertex{80, 0},  vertex{90, 0},  vertex{100, 0}, vertex{110, 0},
        vertex{120, 0}, vertex{130, 0}, vertex{140, 0}, vertex{150, 0}, vertex{160, 0}, vertex{170, 0}};

    auto lanelet =
        std::make_shared<Lanelet>(Lanelet(idLanelet, leftBorderLanelet, rightBorderLanelet,
                                          {LaneletType::mainCarriageWay}, userOneWayLanelet, userBidirectionalLanelet));

    return std::make_shared<RoadNetwork>(RoadNetwork({lanelet}, SupportedTrafficSignCountry::GERMANY));
}

} // namespace utils_predicate_test
