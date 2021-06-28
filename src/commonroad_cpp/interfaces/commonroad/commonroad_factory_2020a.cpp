//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "commonroad_factory_2020a.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"
#include "xml_reader.h"

std::vector<std::shared_ptr<Obstacle>> CommonRoadFactory2020a::createObstacles() {
    std::vector<std::shared_ptr<Obstacle>> obstacleList{};
    pugi::xml_node commonRoad = doc->child("commonRoad");

    // iterate over all nodes and continue working with dynamic and static obstacles
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements != nullptr;
         roadElements = roadElements.next_sibling()) {
        if ((strcmp(roadElements.name(), "dynamicObstacle")) == 0) {
            XMLReader::createDynamicObstacle(obstacleList, roadElements);
        } else if ((strcmp(roadElements.name(), "staticObstacle")) == 0) {
            XMLReader::extractStaticObstacle(obstacleList, roadElements);
        }
        // TODO add environmental and phantom obstacles
    }
    return obstacleList;
}

std::vector<std::shared_ptr<Lanelet>>
CommonRoadFactory2020a::createLanelets(std::vector<std::shared_ptr<TrafficSign>> trafficSigns,
                                       std::vector<std::shared_ptr<TrafficLight>> trafficLights) {

    std::vector<std::shared_ptr<Lanelet>> tempLaneletContainer{};
    pugi::xml_node commonRoad = doc->child("commonRoad");
    XMLReader::initializeLanelets(tempLaneletContainer, commonRoad);

    // get the other values of the lanelets
    size_t arrayIndex{0};
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements != nullptr;
         roadElements = roadElements.next_sibling()) {
        if ((strcmp(roadElements.name(), "lanelet")) == 0) {
            std::set<LaneletType> laneletType;
            std::set<ObstacleType> userOneWay;
            std::set<ObstacleType> userBidirectional;
            for (pugi::xml_node child = roadElements.first_child(); child != nullptr; child = child.next_sibling()) {
                // set left bound
                if ((strcmp(child.name(), "leftBound")) == 0) {
                    XMLReader::extractLaneletBoundary(tempLaneletContainer, arrayIndex, child, "leftBound");
                    continue;
                }
                // set right bound
                if ((strcmp(child.name(), "rightBound")) == 0) {
                    XMLReader::extractLaneletBoundary(tempLaneletContainer, arrayIndex, child, "rightBound");
                    continue;
                }
                // set successor lanelets
                if ((strcmp(child.name(), "successor")) == 0) {
                    XMLReader::extractLaneletPreSuc(tempLaneletContainer, arrayIndex, child, "successor");
                    continue;
                }
                // set predecessor lanelets
                if ((strcmp(child.name(), "predecessor")) == 0) {
                    XMLReader::extractLaneletPreSuc(tempLaneletContainer, arrayIndex, child, "predecessor");
                    continue;
                }
                // set left adjacent lanelets
                if ((strcmp(child.name(), "adjacentLeft")) == 0) {
                    XMLReader::extractLaneletAdjacency(tempLaneletContainer, arrayIndex, child, "adjacentLeft");
                    continue;
                }
                // set right adjacent lanelets
                if ((strcmp(child.name(), "adjacentRight")) == 0) {
                    XMLReader::extractLaneletAdjacency(tempLaneletContainer, arrayIndex, child, "adjacentRight");
                    continue;
                }
                // set lanelet type
                if (!(strcmp(child.name(), "laneletType")))
                    laneletType.insert(lanelet_operations::matchStringToLaneletType(child.first_child().value()));
                // set user one way
                if (!(strcmp(child.name(), "userOneWay")))
                    userOneWay.insert(matchStringToObstacleType(child.first_child().value()));
                // set user bidirectional
                if (!(strcmp(child.name(), "userBidirectional")))
                    userBidirectional.insert(matchStringToObstacleType((child.first_child().value())));
                // add traffic signs to temporary list
                if ((strcmp(child.name(), "trafficSignRef")) == 0) {
                    for (const auto &sign : trafficSigns) {
                        if (child.attribute("ref").as_ullong() == sign->getId()) {
                            tempLaneletContainer[arrayIndex]->addTrafficSign(sign);
                        }
                    }
                }
                // add traffic lights to temporary list
                if ((strcmp(child.name(), "trafficLightRef")) == 0) {
                    for (const auto &light : trafficLights) {
                        if (child.attribute("ref").as_ullong() == light->getId()) {
                            tempLaneletContainer[arrayIndex]->addTrafficLight(light);
                        }
                    }
                }
                // set stop line
                if ((strcmp(child.name(), "stopLine")) == 0) {
                    std::vector<vertex> points;
                    std::shared_ptr<StopLine> sl = std::make_shared<StopLine>();
                    for (pugi::xml_node elem = child.first_child(); elem != nullptr; elem = elem.next_sibling()) {
                        if ((strcmp(elem.name(), "point")) == 0) {
                            vertex newVertice{};
                            newVertice.x = elem.child("x").text().as_double();
                            newVertice.y = elem.child("y").text().as_double();
                            points.push_back(newVertice);
                        }
                        if (!(strcmp(elem.name(), "lineMarking")))
                            sl->setLineMarking(lanelet_operations::matchStringToLineMarking(elem.first_child().value()));
                        if ((strcmp(elem.name(), "trafficSignRef")) == 0) {
                            for (const auto &sign : trafficSigns) {
                                if (child.attribute("ref").as_ullong() == sign->getId()) {
                                    sl->addTrafficSign(sign);
                                }
                            }
                        }
                        if ((strcmp(elem.name(), "trafficLightRef")) == 0) {
                            for (const auto &light : trafficLights) {
                                if (child.attribute("ref").as_ullong() == light->getId()) {
                                    sl->addTrafficLight(light);
                                }
                            }
                        }
                    }
                    sl->setPoints(points);
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
    size_t n{static_cast<size_t>(
        std::distance(commonRoad.children("trafficSign").begin(), commonRoad.children("trafficSign").end()))};
    tempLaneletContainer.clear();
    tempLaneletContainer.reserve(n); // Size already known --> Faster memory allocation

    size_t arrayIndex{0};
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements != nullptr;
         roadElements = roadElements.next_sibling()) {
        // get traffic signs
        if ((strcmp(roadElements.name(), "trafficSign")) == 0) {
            std::shared_ptr<TrafficSign> tempTrafficSign = std::make_shared<TrafficSign>();
            tempLaneletContainer.emplace_back(tempTrafficSign);
            tempLaneletContainer[arrayIndex]->setId(roadElements.first_attribute().as_int());
            for (pugi::xml_node trafficSignChildElement = roadElements.first_child();
                 trafficSignChildElement != nullptr; trafficSignChildElement = trafficSignChildElement.next_sibling()) {
                // get traffic sign elements
                if ((strcmp(trafficSignChildElement.name(), "trafficSignElement")) == 0) {
                    std::string trafficSignId = trafficSignChildElement.first_child().first_child().value();
                    std::shared_ptr<TrafficSignElement> newTrafficSignElement =
                        std::make_shared<TrafficSignElement>(trafficSignId);
                    for (pugi::xml_node trafficSignChildElementChild = trafficSignChildElement.first_child();
                         trafficSignChildElementChild != nullptr;
                         trafficSignChildElementChild = trafficSignChildElementChild.next_sibling()) {
                        if ((strcmp(trafficSignChildElementChild.name(), "additionalValue")) == 0) {
                            newTrafficSignElement->addAdditionalValue(
                                trafficSignChildElementChild.first_child().value());
                        }
                    }
                    tempLaneletContainer[arrayIndex]->addTrafficSignElement(newTrafficSignElement);
                }
                if ((strcmp(trafficSignChildElement.name(), "virtual")) == 0) {
                    tempLaneletContainer[arrayIndex]->setVirtualElement(
                        trafficSignChildElement.first_attribute().as_bool());
                }
                if ((strcmp(trafficSignChildElement.name(), "position")) == 0) {
                    if ((strcmp(trafficSignChildElement.first_child().name(), "point")) == 0) {
                        vertex newVertex{};
                        newVertex.x = trafficSignChildElement.first_child().child("x").text().as_double();
                        newVertex.y = trafficSignChildElement.first_child().child("y").text().as_double();
                        tempLaneletContainer[arrayIndex]->setPosition(newVertex);
                    }
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
    size_t n{static_cast<size_t>(
        std::distance(commonRoad.children("trafficLight").begin(), commonRoad.children("trafficLight").end()))};
    tempLaneletContainer.clear();
    tempLaneletContainer.reserve(n); // Already know the size --> Faster memory allocation

    size_t arrayIndex{0};
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements != nullptr;
         roadElements = roadElements.next_sibling()) {
        // get traffic lights
        if ((strcmp(roadElements.name(), "trafficLight")) == 0) {
            std::shared_ptr<TrafficLight> tempTrafficLight = std::make_shared<TrafficLight>();
            tempLaneletContainer.emplace_back(tempTrafficLight);
            tempLaneletContainer[arrayIndex]->setId(roadElements.first_attribute().as_ullong());
            for (pugi::xml_node trafficLightChildElement = roadElements.first_child();
                 trafficLightChildElement != nullptr;
                 trafficLightChildElement = trafficLightChildElement.next_sibling()) {
                // get traffic light cycle
                if ((strcmp(trafficLightChildElement.name(), "cycle")) == 0) {
                    for (pugi::xml_node trafficLightCycleChildElement = trafficLightChildElement.first_child();
                         trafficLightCycleChildElement != nullptr;
                         trafficLightCycleChildElement = trafficLightCycleChildElement.next_sibling()) {
                        if ((strcmp(trafficLightCycleChildElement.name(), "cycleElement")) == 0) {
                            std::string duration = trafficLightCycleChildElement.first_child().first_child().value();
                            std::string color =
                                trafficLightCycleChildElement.first_child().next_sibling().first_child().value();
                            tempLaneletContainer[arrayIndex]->addCycleElement(
                                {TrafficLight::matchTrafficLightState(color), std::stoul(duration)});
                        }
                        if ((strcmp(trafficLightCycleChildElement.name(), "timeOffset")) == 0) {
                            tempLaneletContainer[arrayIndex]->setOffset(
                                std::stoul(trafficLightCycleChildElement.first_child().value()));
                        }
                    }
                }
                if ((strcmp(trafficLightChildElement.name(), "direction")) == 0) {
                    tempLaneletContainer[arrayIndex]->setDirection(
                        TrafficLight::matchTurningDirections(trafficLightChildElement.first_child().value()));
                }
                if ((strcmp(trafficLightChildElement.name(), "active")) == 0) {
                    tempLaneletContainer[arrayIndex]->setActive(
                        strcasecmp("true", trafficLightChildElement.first_child().value()) == 0);
                }
                if ((strcmp(trafficLightChildElement.name(), "position")) == 0) {
                    if ((strcmp(trafficLightChildElement.first_child().name(), "point")) == 0) {
                        vertex newVertex{};
                        newVertex.x = trafficLightChildElement.first_child().child("x").text().as_double();
                        newVertex.y = trafficLightChildElement.first_child().child("y").text().as_double();
                        tempLaneletContainer[arrayIndex]->setPosition(newVertex);
                    }
                }
            }
            ++arrayIndex;
        }
    }
    return tempLaneletContainer;
}

std::vector<std::shared_ptr<Intersection>>
CommonRoadFactory2020a::createIntersections(const std::vector<std::shared_ptr<Lanelet>> &lanelets) {
    std::vector<std::shared_ptr<Intersection>> tempIntersectionContainer{};
    pugi::xml_node commonRoad = doc->child("commonRoad");

    // get the number of intersections
    size_t n{static_cast<size_t>(
        std::distance(commonRoad.children("intersection").begin(), commonRoad.children("intersection").end()))};
    tempIntersectionContainer.clear();
    tempIntersectionContainer.reserve(n); // Already know the size --> Faster memory allocation

    size_t arrayIndex{0};
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements != nullptr;
         roadElements = roadElements.next_sibling()) {
        if ((strcmp(roadElements.name(), "intersection")) == 0) {
            std::shared_ptr<Intersection> tempIntersection = std::make_shared<Intersection>();
            tempIntersectionContainer.emplace_back(tempIntersection);
            tempIntersectionContainer[arrayIndex]->setId(roadElements.first_attribute().as_int());
            std::map<size_t, size_t> tmpLeftOf;

            for (pugi::xml_node intersectionChildElement = roadElements.first_child();
                 intersectionChildElement != nullptr;
                 intersectionChildElement = intersectionChildElement.next_sibling()) {
                // get incoming
                if ((strcmp(intersectionChildElement.name(), "incoming")) == 0) {
                    std::shared_ptr<Incoming> inc = std::make_shared<Incoming>();
                    inc->setId(intersectionChildElement.first_attribute().as_ullong());
                    std::vector<std::shared_ptr<Lanelet>> incomingLanelet;
                    std::vector<std::shared_ptr<Lanelet>> successorRight;
                    std::vector<std::shared_ptr<Lanelet>> successorStraight;
                    std::vector<std::shared_ptr<Lanelet>> successorLeft;
                    for (pugi::xml_node incomingChildElementChild = intersectionChildElement.first_child();
                         incomingChildElementChild != nullptr;
                         incomingChildElementChild = incomingChildElementChild.next_sibling()) {
                        if ((strcmp(incomingChildElementChild.name(), "incomingLanelet")) == 0) {
                            for (const auto &la : lanelets) {
                                if (incomingChildElementChild.attribute("ref").as_ullong() == la->getId()) {
                                    incomingLanelet.push_back(la);
                                    la->addLaneletType(LaneletType::incoming);
                                }
                            }
                            inc->setIncomingLanelets(incomingLanelet);
                        }
                        if ((strcmp(incomingChildElementChild.name(), "successorsRight")) == 0) {
                            for (const auto &la : lanelets) {
                                if (incomingChildElementChild.attribute("ref").as_ullong() == la->getId())
                                    successorRight.push_back(la);
                            }
                            inc->setSuccessorsRight(successorRight);
                        }
                        if ((strcmp(incomingChildElementChild.name(), "successorsStraight")) == 0) {
                            for (const auto &la : lanelets) {
                                if (incomingChildElementChild.attribute("ref").as_ullong() == la->getId())
                                    successorStraight.push_back(la);
                            }
                            inc->setSuccessorsStraight(successorStraight);
                        }
                        if ((strcmp(incomingChildElementChild.name(), "successorsLeft")) == 0) {
                            for (const auto &la : lanelets) {
                                if (incomingChildElementChild.attribute("ref").as_ullong() == la->getId())
                                    successorLeft.push_back(la);
                            }
                            inc->setSuccessorsLeft(successorLeft);
                        }
                        if (!(strcmp(incomingChildElementChild.name(), "isLeftOf")))
                            tmpLeftOf.insert_or_assign(inc->getId(),
                                                       incomingChildElementChild.attribute("ref").as_ullong());
                    }
                    tempIntersectionContainer[arrayIndex]->addIncoming(inc);
                }
                if ((strcmp(intersectionChildElement.name(), "crossing")) == 0) {
                    for (pugi::xml_node crossingElement = intersectionChildElement.first_child();
                         crossingElement != nullptr; crossingElement = crossingElement.next_sibling()) {
                    }
                }
            }
            // iterate over all incoming lefts and assign correct reference
            for (auto const &[key, val] : tmpLeftOf) {
                for (const auto &inc1 : tempIntersectionContainer[arrayIndex]->getIncomings()) {
                    if (inc1->getId() == key)
                        for (const auto &inc2 : tempIntersectionContainer[arrayIndex]->getIncomings()) {
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
