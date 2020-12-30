//
// Created by Sebastian Maierhofer on 04.11.20.
//

#include "commonroad_factory_2020a.h"
#include "xml_reader.h"
#include "../../roadNetwork/lanelet/lanelet_operations.h"
#include "../../obstacle/obstacle_operations.h"


std::vector<std::shared_ptr<Obstacle>> CommonRoadFactory2020a::createObstacles() {
    std::vector<std::shared_ptr<Obstacle>> obstacleList{};
    pugi::xml_node commonRoad = doc->child("commonRoad");

    // iterate over all nodes and continue working with dynamic and static obstacles
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements;
         roadElements = roadElements.next_sibling()) {
        if (!(strcmp(roadElements.name(), "dynamicObstacle"))) {
            XMLReader::createDynamicObstacle(obstacleList, roadElements);
        } else if (!(strcmp(roadElements.name(), "staticObstacle"))) {
            XMLReader::extractStaticObstacle(obstacleList, roadElements);
        }
    }
    return obstacleList;
}


std::vector<std::shared_ptr<Lanelet>> CommonRoadFactory2020a::createLanelets(
        std::vector<std::shared_ptr<TrafficSign>> trafficSigns,
        std::vector<std::shared_ptr<TrafficLight>> trafficLights) {

    std::vector<std::shared_ptr<Lanelet>> tempLaneletContainer{};
    pugi::xml_node commonRoad = doc->child("commonRoad");
    XMLReader::initializeLanelets(tempLaneletContainer, commonRoad);

    // get the other values of the lanelets
    int arrayIndex{0};
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements;
         roadElements = roadElements.next_sibling()) {
        if (!(strcmp(roadElements.name(), "lanelet"))) {
            std::vector<LaneletType> laneletType;
            std::vector<ObstacleType> userOneWay;
            std::vector<ObstacleType> userBidirectional;
            std::vector<std::shared_ptr<TrafficSign>> signs;
            std::vector<std::shared_ptr<TrafficLight>> lights;
            for (pugi::xml_node child = roadElements.first_child(); child; child = child.next_sibling()) {
                // set left bound
                if (!(strcmp(child.name(), "leftBound"))) {
                    XMLReader::extractLaneletBoundary(tempLaneletContainer, arrayIndex, child, "leftBound");
                    continue;
                }
                // set right bound
                if (!(strcmp(child.name(), "rightBound"))) {
                    XMLReader::extractLaneletBoundary(tempLaneletContainer, arrayIndex, child, "rightBound");
                    continue;
                }
                // set successor lanelets
                if (!(strcmp(child.name(), "successor"))) {
                    XMLReader::extractLaneletPreSuc(tempLaneletContainer, arrayIndex, child, "successor");
                    continue;
                }
                // set predecessor lanelets
                if (!(strcmp(child.name(), "predecessor"))) {
                    XMLReader::extractLaneletPreSuc(tempLaneletContainer, arrayIndex, child, "predecessor");
                    continue;
                }
                // set left adjacent lanelets
                if (!(strcmp(child.name(), "adjacentLeft"))) {
                    XMLReader::extractLaneletAdjacency(tempLaneletContainer, arrayIndex, child, "adjacentLeft");
                    continue;
                }
                // set right adjacent lanelets
                if (!(strcmp(child.name(), "adjacentRight"))) {
                    XMLReader::extractLaneletAdjacency(tempLaneletContainer, arrayIndex, child, "adjacentRight");
                    continue;
                }
                // set lanelet type
                if (!(strcmp(child.name(), "laneletType")))
                    laneletType.push_back(matchStringToLaneletType(child.first_child().value()));
                // set user one way
                if (!(strcmp(child.name(), "userOneWay")))
                    userOneWay.push_back(matchObstacleTypeToString(child.first_child().value()));
                // set user bidirectional
                if (!(strcmp(child.name(), "userBidirectional")))
                    userBidirectional.push_back(matchObstacleTypeToString(child.first_child().value()));
                // add traffic signs to temporary list
                if (!(strcmp(child.name(), "trafficSign"))) {
                    for (const auto &sign: trafficSigns) {
                        if (child.attribute("ref").as_int() == sign->getId()) {
                            signs.push_back(sign);
                        }
                    }
                }
                // add traffic lights to temporary list
                if (!(strcmp(child.name(), "trafficLight"))) {
                    for (const auto &light: trafficLights) {
                        if (child.attribute("ref").as_int() == light->getId()) {
                            lights.push_back(light);
                        }
                    }
                }
                // set stop line
                if (!(strcmp(child.name(), "stopLine"))) {
                    std::vector<vertex> points;
                    StopLine sl = StopLine();
                    for (pugi::xml_node elem = child.first_child(); elem; elem = elem.next_sibling()) {
                        if (!(strcmp(elem.name(), "point"))) {
                            vertex newVertice{};
                            newVertice.x = elem.child("x").text().as_double();
                            newVertice.y = elem.child("y").text().as_double();
                            points.push_back(newVertice);
                        }
                        if (!(strcmp(elem.name(), "lineMarking")))
                            sl.setLineMarking(matchStringToLineMarking(elem.first_child().value()));
                        if (!(strcmp(elem.name(), "trafficSignRef"))) {
                            for (const auto &sign: trafficSigns) {
                                if (child.attribute("ref").as_int() == sign->getId()) {
                                    sl.setTrafficSign(sign);
                                }
                            }
                        }
                        if (!(strcmp(elem.name(), "trafficLightRef"))) {
                            for (const auto &light: trafficLights) {
                                if (child.attribute("ref").as_int() == light->getId()) {
                                    sl.setTrafficLight(light);
                                }
                            }
                        }
                    }
                    sl.setPoints(points);
                    tempLaneletContainer[arrayIndex]->setStopLine(sl);
                }
            }
            tempLaneletContainer[arrayIndex]->createCenterVertices();
            tempLaneletContainer[arrayIndex]->constructOuterPolygon();
            tempLaneletContainer[arrayIndex]->setLaneletType(laneletType);
            tempLaneletContainer[arrayIndex]->setUserOneWay(userOneWay);
            tempLaneletContainer[arrayIndex]->setUserBidirectional(userBidirectional);
            arrayIndex++;
        }
    }
    return tempLaneletContainer;
}

std::vector<std::shared_ptr<TrafficSign>> CommonRoadFactory2020a::createTrafficSigns() {
    std::vector<std::shared_ptr<TrafficSign>> tempLaneletContainer{};
    pugi::xml_node commonRoad = doc->child("commonRoad");

    // get the number of traffic signs in scenario
    int n = std::distance(commonRoad.children("trafficSign").begin(),
                          commonRoad.children("trafficSign").end());
    tempLaneletContainer.clear();
    tempLaneletContainer.reserve(n); // Size already known --> Faster memory allocation

    int arrayIndex = 0;
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements;
         roadElements = roadElements.next_sibling()) {
        //get traffic signs
        if (!(strcmp(roadElements.name(), "trafficSign"))) {
            std::shared_ptr<TrafficSign> tempTrafficSign = std::make_shared<TrafficSign>();
            tempLaneletContainer.emplace_back(tempTrafficSign);
            tempLaneletContainer[arrayIndex]->setId(roadElements.first_attribute().as_int());
            for (pugi::xml_node trafficSignChildElement = roadElements.first_child(); trafficSignChildElement;
                 trafficSignChildElement = trafficSignChildElement.next_sibling()) {
                // get traffic sign elements
                if (!(strcmp(trafficSignChildElement.name(), "trafficSignElement"))) {
                    std::string trafficSignId = trafficSignChildElement.first_child().first_child().value();
                    TrafficSignElement newTrafficSignElement = TrafficSignElement(trafficSignId);
                    std::vector<std::string> additionalValuesList;
                    for (pugi::xml_node trafficSignChildElementChild = trafficSignChildElement.first_child();
                         trafficSignChildElementChild;
                         trafficSignChildElementChild = trafficSignChildElementChild.next_sibling()) {
                        if (!(strcmp(trafficSignChildElementChild.name(), "additionalValue"))) {
                            newTrafficSignElement.addAdditionalValue(
                                    trafficSignChildElementChild.first_child().value());
                        }
                    }
                    tempLaneletContainer[arrayIndex]->addTrafficSignElement(newTrafficSignElement);
                }
                if (!(strcmp(trafficSignChildElement.name(), "virtual"))) {
                    tempLaneletContainer[arrayIndex]->setVirtualElement(
                            trafficSignChildElement.first_attribute().as_bool());
                }
            }
            ++arrayIndex;
        }
    }
    return tempLaneletContainer;
}

std::vector<std::shared_ptr<TrafficLight>> CommonRoadFactory2020a::createTrafficLights() {
    std::vector<std::shared_ptr<TrafficLight>> tempLaneletContainer{};
    pugi::xml_node commonRoad = doc->child("commonRoad");

    // get the number of traffic lights
    int n = std::distance(commonRoad.children("trafficLight").begin(),
                          commonRoad.children("trafficLight").end());
    tempLaneletContainer.clear();
    tempLaneletContainer.reserve(n); // Already know the size --> Faster memory allocation

    int arrayIndex = 0;

    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements;
         roadElements = roadElements.next_sibling()) {
        // get traffic lights
        if (!(strcmp(roadElements.name(), "trafficLight"))) {
            std::shared_ptr<TrafficLight> tempTrafficLight = std::make_shared<TrafficLight>();
            tempLaneletContainer.emplace_back(tempTrafficLight);
            tempLaneletContainer[arrayIndex]->setId(roadElements.first_attribute().as_int());
            for (pugi::xml_node trafficLightChildElement = roadElements.first_child(); trafficLightChildElement;
                 trafficLightChildElement = trafficLightChildElement.next_sibling()) {
                // get traffic light cycle
                if (!(strcmp(trafficLightChildElement.name(), "cycle"))) {
                    for (pugi::xml_node trafficLightCycleChildElement = trafficLightChildElement.first_child();
                         trafficLightCycleChildElement;
                         trafficLightCycleChildElement = trafficLightCycleChildElement.next_sibling()) {
                        if (!(strcmp(trafficLightCycleChildElement.name(), "cycleElement"))) {
                            std::string duration = trafficLightCycleChildElement.first_child().first_child().value();
                            std::string color =
                                    trafficLightCycleChildElement.first_child().next_sibling().first_child().value();
                            CycleElement cycle{};
                            if (color == "red")
                                cycle = CycleElement{CycleElementType::red, std::stoi(duration)};
                            if (color == "green")
                                cycle = CycleElement{CycleElementType::green, std::stoi(duration)};
                            if (color == "yellow")
                                cycle = CycleElement{CycleElementType::yellow, std::stoi(duration)};
                            if (color == "red_yellow")
                                cycle = CycleElement{CycleElementType::red_yellow, std::stoi(duration)};

                            tempLaneletContainer[arrayIndex]->addCycleElement(cycle);
                        }
                        if (!(strcmp(trafficLightCycleChildElement.name(), "timeOffset"))) {
                            tempLaneletContainer[arrayIndex]->setOffset(
                                    std::stoi(trafficLightCycleChildElement.first_child().value()));
                        }
                    }
                }
                if (!(strcmp(trafficLightChildElement.name(), "direction"))) {
                    TrafficLightDirection dir;
                    if (!(strcmp(trafficLightChildElement.first_child().value(), "right")))
                        dir = TrafficLightDirection::right;
                    if (!(strcmp(trafficLightChildElement.first_child().value(), "straight")))
                        dir = TrafficLightDirection::straight;
                    if (!(strcmp(trafficLightChildElement.first_child().value(), "left")))
                        dir = TrafficLightDirection::left;
                    if (!(strcmp(trafficLightChildElement.first_child().value(), "leftStraight")))
                        dir = TrafficLightDirection::leftStraight;
                    if (!(strcmp(trafficLightChildElement.first_child().value(), "straightRight")))
                        dir = TrafficLightDirection::straightRight;
                    if (!(strcmp(trafficLightChildElement.first_child().value(), "leftRight")))
                        dir = TrafficLightDirection::leftRight;
                    if (!(strcmp(trafficLightChildElement.first_child().value(), "all")))
                        dir = TrafficLightDirection::all;
                    tempLaneletContainer[arrayIndex]->setDirection(dir);
                }
                if (!(strcmp(trafficLightChildElement.name(), "active"))) {
                    tempLaneletContainer[arrayIndex]->setActive(
                            strcasecmp("true", trafficLightChildElement.first_child().value()) == 0);
                }
            }
            ++arrayIndex;
        }
    }
    return tempLaneletContainer;
}

std::vector<std::shared_ptr<Intersection>> CommonRoadFactory2020a::createIntersections(
        const std::vector<std::shared_ptr<Lanelet>> &lanelets) {
    std::vector<std::shared_ptr<Intersection>> tempIntersectionContainer{};
    pugi::xml_node commonRoad = doc->child("commonRoad");

    // get the number of intersections
    int n = std::distance(commonRoad.children("intersection").begin(),
                          commonRoad.children("intersection").end());
    tempIntersectionContainer.clear();
    tempIntersectionContainer.reserve(n); // Already know the size --> Faster memory allocation

    int arrayIndex = 0;
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements;
         roadElements = roadElements.next_sibling()) {
        if (!(strcmp(roadElements.name(), "intersection"))) {
            std::shared_ptr<Intersection> tempIntersection = std::make_shared<Intersection>();
            tempIntersectionContainer.emplace_back(tempIntersection);
            tempIntersectionContainer[arrayIndex]->setId(roadElements.first_attribute().as_int());
            std::map<int, int> tmpLeftOf;

            for (pugi::xml_node intersectionChildElement = roadElements.first_child(); intersectionChildElement;
                 intersectionChildElement = intersectionChildElement.next_sibling()) {
                // get incoming
                if (!(strcmp(intersectionChildElement.name(), "incoming"))) {
                    std::shared_ptr<Incoming> inc = std::make_shared<Incoming>();
                    inc->setId(intersectionChildElement.first_attribute().as_int());
                    std::vector<std::shared_ptr<Lanelet>> incomingLanelet;
                    std::vector<std::shared_ptr<Lanelet>> successorRight;
                    std::vector<std::shared_ptr<Lanelet>> successorStraight;
                    std::vector<std::shared_ptr<Lanelet>> successorLeft;
                    for (pugi::xml_node incomingChildElementChild = intersectionChildElement.first_child();
                         incomingChildElementChild;
                         incomingChildElementChild = incomingChildElementChild.next_sibling()) {
                        if (!(strcmp(incomingChildElementChild.name(), "incomingLanelet"))) {
                            for (const auto &la: lanelets) {
                                if (incomingChildElementChild.attribute("ref").as_int() == la->getId())
                                    incomingLanelet.push_back(la);
                            }
                            inc->setIncomingLanelet(incomingLanelet);
                        }
                        if (!(strcmp(incomingChildElementChild.name(), "successorsRight"))) {
                            for (const auto &la: lanelets) {
                                if (incomingChildElementChild.attribute("ref").as_int() == la->getId())
                                    successorRight.push_back(la);
                            }
                            inc->setSuccessorsRight(successorRight);
                        }
                        if (!(strcmp(incomingChildElementChild.name(), "successorsStraight"))) {
                            for (const auto &la: lanelets) {
                                if (incomingChildElementChild.attribute("ref").as_int() == la->getId())
                                    successorStraight.push_back(la);
                            }
                            inc->setSuccessorsStraight(successorStraight);
                        }
                        if (!(strcmp(incomingChildElementChild.name(), "successorsLeft"))) {
                            for (const auto &la: lanelets) {
                                if (incomingChildElementChild.attribute("ref").as_int() == la->getId())
                                    successorLeft.push_back(la);
                            }
                            inc->setSuccessorsLeft(successorLeft);
                        }
                        if (!(strcmp(incomingChildElementChild.name(), "isLeftOf")))
                            tmpLeftOf.insert_or_assign(inc->getId(),
                                                       incomingChildElementChild.attribute("ref").as_int());
                    }
                    tempIntersectionContainer[arrayIndex]->addIncoming(inc);
                }
                if (!(strcmp(intersectionChildElement.name(), "crossing"))) {
                    for (pugi::xml_node crossingElement = intersectionChildElement.first_child(); crossingElement;
                         crossingElement = crossingElement.next_sibling()) {

                    }
                }
            }
            // iterate over all incoming lefts and assign correct reference
            for (auto const&[key, val] : tmpLeftOf) {
                for (const auto &inc1 : tempIntersectionContainer[arrayIndex]->getIncoming()) {
                    if (inc1->getId() == key)
                        for (const auto &inc2 : tempIntersectionContainer[arrayIndex]->getIncoming()) {
                            if (inc2->getId() == val)
                                inc1->setIsLeftOf(inc2);
                        }
                }
            }
            ++arrayIndex;
        }
    }
    return tempIntersectionContainer;
}


