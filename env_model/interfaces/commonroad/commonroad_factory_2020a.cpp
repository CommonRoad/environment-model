//
// Created by Sebastian Maierhofer on 04.11.20.
//

#include "commonroad_factory_2020a.h"
#include "xml_reader.h"
#include "../../road_network/regulatory_elements/stop_line.h"
#include "../../obstacle/obstacle_operations.h"
#include <cstdlib>

std::vector<std::shared_ptr<Obstacle>> CommonRoadFactory2020a::createObstacles() {
    std::vector<std::shared_ptr<Obstacle>> obstacleList{};
    pugi::xml_node commonRoad = doc->child("commonRoad");

    // iterate over all nodes and continue working with dynamic and static obstacles
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements;
         roadElements = roadElements.next_sibling()) {
        if (!(strcmp(roadElements.name(), "dynamicObstacle"))) {
            std::shared_ptr<Obstacle> tempObstacle(nullptr); // Empty pointer (specific object gets assigned in the following)
            tempObstacle = std::make_shared<Obstacle>();

            // extract ID, type, shape, initial state, and trajectory
            tempObstacle->setId(roadElements.first_attribute().as_int());
            tempObstacle->setType(matchObstacleTypeToString(roadElements.first_child().text().as_string()));
            for (pugi::xml_node child = roadElements.first_child(); child; child = child.next_sibling()) {
                if (!(strcmp(child.name(), "shape"))) {
                    if (!(strcmp(child.first_child().name(), "rectangle"))) {
                        tempObstacle->getGeoShape().setLength(child.first_child().child("length").text().as_double());
                        tempObstacle->getGeoShape().setWidth(child.first_child().child("width").text().as_double());
                    }
                    continue;
                }
                if (!(strcmp(child.name(), "initialState"))) {
                    State initialState = XMLReader::extractInitialState(child);
                    tempObstacle->setCurrentState(initialState);
                }
                if (!(strcmp(child.name(), "trajectory"))) {
                    for (pugi::xml_node states = child.first_child(); states; states = states.next_sibling()) {
                        State st = XMLReader::extractState(states);
                        tempObstacle->appendState(st);
                    }
                }
            }
            obstacleList.emplace_back(tempObstacle);
        }
        else if  (!(strcmp(roadElements.name(), "staticObstacle"))) {
			std::shared_ptr<Obstacle> tempObstacle(nullptr); // Empty pointer (specific object gets assigned in the following)
			tempObstacle = std::make_shared<Obstacle>();

            // extract ID, type, shape, and initial state
			tempObstacle->setId(roadElements.first_attribute().as_int());
            tempObstacle->setIsStatic(true);
            tempObstacle->setType(matchObstacleTypeToString(roadElements.first_child().text().as_string()));
			for (pugi::xml_node child = roadElements.first_child(); child; child = child.next_sibling()) {
                if (!(strcmp(child.name(), "shape"))) {
                    if (!(strcmp(child.first_child().name(), "rectangle"))) {
                        tempObstacle->getGeoShape().setLength(child.first_child().child("length").text().as_double());
                        tempObstacle->getGeoShape().setWidth(child.first_child().child("width").text().as_double());
                    }
                    continue;
                }
                if (!(strcmp(child.name(), "initialState"))) {
                    State initialState = XMLReader::extractInitialState(child);
                    tempObstacle->setCurrentState(initialState);
                }
            }
			obstacleList.emplace_back(tempObstacle);
		}
    }
    return obstacleList;
}


std::vector<std::shared_ptr<Lanelet>> CommonRoadFactory2020a::createLanelets(std::vector<std::shared_ptr<TrafficSign>> trafficSigns, std::vector<std::shared_ptr<TrafficLight>> trafficLights) {
    std::vector<std::shared_ptr<Lanelet>> tempLaneletContainer{};
    pugi::xml_node commonRoad = doc->child("commonRoad");

    // get the number of lanelets
    size_t n = std::distance(commonRoad.children("lanelet").begin(), commonRoad.children("lanelet").end());
    tempLaneletContainer.clear();
    tempLaneletContainer.reserve(n); // Already know the size --> Faster memory allocation

    // all lanelets must be initialized first because they are referencing each other
    for (size_t i = 0; i < n; i++) {
        Lanelet newLanelet;
        std::shared_ptr<Lanelet> tempLanelet = std::make_shared<Lanelet>(); // make_shared is faster than (new vehicularLanelet());
        tempLaneletContainer.emplace_back(tempLanelet);
    }

    size_t arrayIndex = 0;
    // set id of lanelets
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements;
         roadElements = roadElements.next_sibling()) {
        if (!(strcmp(roadElements.name(), "lanelet"))) {
            tempLaneletContainer[arrayIndex]->setId(roadElements.first_attribute().as_int());
            arrayIndex++;
        }
    }

    // get the other values of the lanelets
    arrayIndex = 0;
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
                    for (pugi::xml_node points = child.first_child(); points; points = points.next_sibling()) {
                        if (!(strcmp(points.name(), "point"))) {
                            vertice newVertice{};
                            newVertice.x = points.child("x").text().as_double();
                            newVertice.y = points.child("y").text().as_double();
                            tempLaneletContainer[arrayIndex]->addLeftVertice(newVertice);
                        }
                    }
                    continue;
                }
                // set right bound
                if (!(strcmp(child.name(), "rightBound"))) {
                    for (pugi::xml_node points = child.first_child(); points; points = points.next_sibling()) {
                        if (!(strcmp(points.name(), "point"))) {
                            vertice newVertice{};
                            newVertice.x = points.child("x").text().as_double();
                            newVertice.y = points.child("y").text().as_double();
                            tempLaneletContainer[arrayIndex]->addRightVertice(newVertice);
                        }
                    }
                    continue;
                }
                // set successor lanelets
                if (!(strcmp(child.name(), "successor"))) {
                    size_t successorId = child.first_attribute().as_int();
                    for (size_t i = 0; i < n; i++) {
                        if (tempLaneletContainer[i]->getId() == successorId) {
                            tempLaneletContainer[arrayIndex]->addSuccessor(tempLaneletContainer[i].get());
                            break;
                        }
                    }
                    continue;
                }
                // set predecessor lanelets
                if (!(strcmp(child.name(), "predecessor"))) {
                    size_t predecessorId = child.first_attribute().as_int();
                    for (size_t i = 0; i < n; i++) {
                        if (tempLaneletContainer[i]->getId() == predecessorId) {
                            tempLaneletContainer[arrayIndex]->addPredecessor(tempLaneletContainer[i].get());
                            break;
                        }
                    }
                    continue;
                }
                // set left adjacent lanelets
                if (!(strcmp(child.name(), "adjacentLeft"))) {
                    size_t adjacentId = child.attribute("ref").as_int();
                    std::string dir = child.attribute("drivingDir").as_string();
                    for (size_t i = 0; i < n; i++) {
                        if (tempLaneletContainer[i]->getId() == adjacentId) {
                            tempLaneletContainer[arrayIndex]->setLeftAdjacent(tempLaneletContainer[i].get(), dir);
                            break;
                        }
                    }
                    continue;
                }
                // set right adjacent lanelets
                if (!(strcmp(child.name(), "adjacentRight"))) {
                    size_t adjacentId = child.attribute("ref").as_int();
                    std::string dir = child.attribute("drivingDir").as_string();
                    for (size_t i = 0; i < n; i++) {
                        if (tempLaneletContainer[i]->getId() == adjacentId) {
                            tempLaneletContainer[arrayIndex]->setRightAdjacent(tempLaneletContainer[i].get(), dir);
                            break;
                        }
                    }
                    continue;
                }
                // set lanelet type
                if (!(strcmp(child.name(), "laneletType")))
                    laneletType.push_back(matchLaneletTypeToString(child.first_child().value()));
                // set user one way
                if (!(strcmp(child.name(), "userOneWay")))
                    userOneWay.push_back(matchObstacleTypeToString(child.first_child().value()));
                // set user bidirectional
                if (!(strcmp(child.name(), "userBidirectional")))
                    userBidirectional.push_back(matchObstacleTypeToString(child.first_child().value()));
                // add traffic signs to temporary list
                if (!(strcmp(child.name(), "trafficSign"))) {
                    for(const auto& sign: trafficSigns){
                        if (child.attribute("ref").as_int() == sign->getId()) {
                            signs.push_back(sign);
                        }
                    }
                }
                // add traffic lights to temporary list
                if (!(strcmp(child.name(), "trafficLight"))) {
                    for(const auto& light: trafficLights){
                        if (child.attribute("ref").as_int() == light->getId()) {
                            lights.push_back(light);
                        }
                    }
                }
                // set stop line
                if (!(strcmp(child.name(), "stopLine"))) {
                    std::vector<vertice> points;
                    StopLine sl = StopLine();
                    for (pugi::xml_node elem = child.first_child(); elem; elem = elem.next_sibling()) {
                        if (!(strcmp(elem.name(), "point"))) {
                            vertice newVertice{};
                            newVertice.x = elem.child("x").text().as_double();
                            newVertice.y = elem.child("y").text().as_double();
                            points.push_back(newVertice);
                        }
                        if (!(strcmp(elem.name(), "lineMarking")))
                            sl.setLineMarking(matchLineMarkingToString(elem.first_child().value()));
                        if (!(strcmp(elem.name(), "trafficSignRef"))) {
                            for(const auto& sign: trafficSigns){
                                if (child.attribute("ref").as_int() == sign->getId()) {
                                    sl.setTrafficSign(sign);
                                }
                            }
                        }
                        if (!(strcmp(elem.name(), "trafficLightRef"))) {
                            for(const auto& light: trafficLights){
                                if (child.attribute("ref").as_int() == light->getId()) {
                                    sl.setTrafficLight(light);
                                }
                            }
                        }
                    }
                    sl.setPoints(points);
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

    // get the number of traffic signs
    size_t n = std::distance(commonRoad.children("trafficSign").begin(), commonRoad.children("trafficSign").end());
    tempLaneletContainer.clear();
    tempLaneletContainer.reserve(n); // Already know the size --> Faster memory allocation

    size_t arrayIndex = 0;
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
                    std::vector<std::string > additionalValuesList;
                    for (pugi::xml_node trafficSignChildElementChild = trafficSignChildElement.first_child(); trafficSignChildElementChild;
                         trafficSignChildElementChild = trafficSignChildElementChild.next_sibling()) {
                        if (!(strcmp(trafficSignChildElementChild.name(), "additionalValue"))) {
                            newTrafficSignElement.addAdditionalValue(trafficSignChildElementChild.first_child().value());
                        }
                    }
                    tempLaneletContainer[arrayIndex]->addTrafficSignElement(newTrafficSignElement); //TODO pass by reference
                }
                if (!(strcmp(trafficSignChildElement.name(), "virtual"))) {
                    tempLaneletContainer[arrayIndex]->setVirtualElement(trafficSignChildElement.first_attribute().as_bool());
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
    size_t n = std::distance(commonRoad.children("trafficLight").begin(), commonRoad.children("trafficLight").end());
    tempLaneletContainer.clear();
    tempLaneletContainer.reserve(n); // Already know the size --> Faster memory allocation

    size_t arrayIndex = 0;

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
                    for (pugi::xml_node trafficLightCycleChildElement = trafficLightChildElement.first_child(); trafficLightCycleChildElement;
                         trafficLightCycleChildElement = trafficLightCycleChildElement.next_sibling()) {
                        if (!(strcmp(trafficLightCycleChildElement.name(), "cycleElement"))) {
                            std::string duration = trafficLightCycleChildElement.first_child().first_child().value();
                            std::string color = trafficLightCycleChildElement.first_child().next_sibling().first_child().value();
                            CycleElement cycle{};
                            if (color == "red")
                                cycle = CycleElement{CycleElementType::red, std::stof(duration) };
                            if (color == "green")
                                cycle = CycleElement{CycleElementType::green, std::stof(duration) };
                            if (color == "yellow")
                                cycle = CycleElement{CycleElementType::yellow, std::stof(duration) };
                            if (color == "red_yellow")
                                cycle = CycleElement{CycleElementType::red_yellow, std::stof(duration) };

                            tempLaneletContainer[arrayIndex]->addCycleElement(cycle); //TODO pass by reference
                        }
                        if (!(strcmp(trafficLightCycleChildElement.name(), "timeOffset"))) {
                            tempLaneletContainer[arrayIndex]->setOffset(std::stoi(trafficLightCycleChildElement.first_child().value()));
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
                    tempLaneletContainer[arrayIndex]->setActive(strcasecmp("true", trafficLightChildElement.first_child().value()) == 0);
                }
            }
            ++arrayIndex;
        }
    }
    return tempLaneletContainer;
}

std::vector<std::shared_ptr<Intersection>> CommonRoadFactory2020a::createIntersections(const std::vector<std::shared_ptr<Lanelet>>& lanelets) {
    std::vector<std::shared_ptr<Intersection>> tempIntersectionContainer{};
    pugi::xml_node commonRoad = doc->child("commonRoad");

    // get the number of intersections
    size_t n = std::distance(commonRoad.children("intersection").begin(), commonRoad.children("intersection").end());
    tempIntersectionContainer.clear();
    tempIntersectionContainer.reserve(n); // Already know the size --> Faster memory allocation

    size_t arrayIndex = 0;
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements;
         roadElements = roadElements.next_sibling()) {
        if (!(strcmp(roadElements.name(), "intersection"))) {
            std::shared_ptr<Intersection> tempIntersection = std::make_shared<Intersection>();
            tempIntersectionContainer.emplace_back(tempIntersection);
            tempIntersectionContainer[arrayIndex]->setId(roadElements.first_attribute().as_int());
            std::map<int, int> tmpLeftOf;

            for (pugi::xml_node intersectionChildElement = roadElements.first_child(); intersectionChildElement;
                 intersectionChildElement = intersectionChildElement.next_sibling()) {
                // get incomings
                if (!(strcmp(intersectionChildElement.name(), "incoming"))) {
                    std::shared_ptr<Incoming> inc = std::make_shared<Incoming>();
                    inc->setId(intersectionChildElement.first_attribute().as_int());
                    std::vector<std::shared_ptr<Lanelet>> incomingLanelet;
                    std::vector<std::shared_ptr<Lanelet>> successorRight;
                    std::vector<std::shared_ptr<Lanelet>> successorStraight;
                    std::vector<std::shared_ptr<Lanelet>> successorLeft;
                    for (pugi::xml_node incomingChildElementChild = intersectionChildElement.first_child(); incomingChildElementChild;
                         incomingChildElementChild = incomingChildElementChild.next_sibling()) {
                        if (!(strcmp(incomingChildElementChild.name(), "incomingLanelet"))) {
                            for(const auto& la: lanelets){
                                if (incomingChildElementChild.attribute("ref").as_int() == la->getId())
                                    incomingLanelet.push_back(la);
                            }
                            inc->setIncomingLanelet(incomingLanelet);
                        }
                        if (!(strcmp(incomingChildElementChild.name(), "successorsRight"))) {
                            for(const auto& la: lanelets){
                                if (incomingChildElementChild.attribute("ref").as_int() == la->getId())
                                    successorRight.push_back(la);
                            }
                            inc->setSuccessorsRight(successorRight);
                        }
                        if (!(strcmp(incomingChildElementChild.name(), "successorsStraight"))) {
                            for(const auto& la: lanelets){
                                if (incomingChildElementChild.attribute("ref").as_int() == la->getId())
                                    successorStraight.push_back(la);
                            }
                            inc->setSuccessorsStraight(successorStraight);
                        }
                        if (!(strcmp(incomingChildElementChild.name(), "successorsLeft"))) {
                            for(const auto& la: lanelets){
                                if (incomingChildElementChild.attribute("ref").as_int() == la->getId())
                                    successorLeft.push_back(la);
                            }
                            inc->setSuccessorsLeft(successorLeft);
                        }
                        if (!(strcmp(incomingChildElementChild.name(), "isLeftOf")))
                            tmpLeftOf.insert_or_assign(inc->getId(), incomingChildElementChild.attribute("ref").as_int());
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
            for(auto const& [key, val] : tmpLeftOf){
                for(const auto& inc1 : tempIntersectionContainer[arrayIndex]->getIncoming()){
                    if(inc1->getId() == key)
                        for(const auto& inc2 : tempIntersectionContainer[arrayIndex]->getIncoming()) {
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
