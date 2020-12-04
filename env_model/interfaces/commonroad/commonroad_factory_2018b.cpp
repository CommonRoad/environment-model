//
// Created by Sebastian Maierhofer on 08.11.20.
//

#include "commonroad_factory_2018b.h"
#include "../../obstacle/obstacle_operations.h"
#include "xml_reader.h"

std::vector<std::shared_ptr<Obstacle>> CommonRoadFactory2018b::createObstacles() {
    std::vector<std::shared_ptr<Obstacle>> obstacleList{};
    pugi::xml_node commonRoad = doc->child("commonRoad");

    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements;
         roadElements = roadElements.next_sibling()) {
        if (!(strcmp(roadElements.name(), "Obstacle"))) {
            if (!(strcmp(roadElements.first_child().text().as_string(), "dynamic"))){
                std::shared_ptr<Obstacle> tempObstacle(nullptr); // Empty pointer (specific object gets assigned in the following)
                tempObstacle = std::make_shared<Obstacle>();

                // extract ID, type, shape, initial state, and trajectory
                tempObstacle->setId(roadElements.first_attribute().as_int());
                tempObstacle->setType(matchObstacleTypeToString(roadElements.first_child().text().as_string()));
                for (pugi::xml_node child = roadElements.first_child(); child; child = child.next_sibling()) {
                    if (!(strcmp(child.name(), "shape"))) {
                        if (!(strcmp(child.first_child().name(), "rectangle"))) {
                            tempObstacle->getGeoShape().setLength(
                                    child.first_child().child("length").text().as_double());
                            tempObstacle->getGeoShape().setWidth(child.first_child().child("width").text().as_double());
                        }
                        continue;
                    }
                    if (!(strcmp(child.name(), "initialState"))) {
                        State initialState = XMLReader::extractInitialState(child);
                        tempObstacle->setCurrentState(initialState);
                        tempObstacle->appendState(initialState);
                    } else if (!(strcmp(child.name(), "trajectory"))) {
                        for (pugi::xml_node states = child.first_child(); states; states = states.next_sibling()) {
                            State st = XMLReader::extractState(states);
                            tempObstacle->appendState(st);
                        }
                    }
                }
                obstacleList.emplace_back(tempObstacle);
            }
            else if (!(strcmp(roadElements.first_child().text().as_string(), "static"))) {
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
                    } else if (!(strcmp(child.name(), "initialState"))) {
                        State initialState = XMLReader::extractInitialState(child);
                        tempObstacle->setCurrentState(initialState);
                    }
                }
                obstacleList.emplace_back(tempObstacle);
            }
        }
    }
    return obstacleList;
}

// TODO: extract speed limit signs from lanelet
std::vector<std::shared_ptr<Lanelet>> CommonRoadFactory2018b::createLanelets(std::vector<std::shared_ptr<TrafficSign>> trafficSigns, std::vector<std::shared_ptr<TrafficLight>> trafficLights) {
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
    // set id of the lanelets
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
            for (pugi::xml_node child = roadElements.first_child(); child; child = child.next_sibling()) {
                // set left bound
                if (!(strcmp(child.name(), "leftBound"))) {
                    for (pugi::xml_node points = child.first_child(); points; points = points.next_sibling()) {
                        if (!(strcmp(points.name(), "point"))) {
                            vertice newVertice{};
                            newVertice.x = points.child("x").text().as_double();
                            newVertice.y = points.child("y").text().as_double();
                            tempLaneletContainer[arrayIndex]->addLeftVertex(newVertice);
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
                            tempLaneletContainer[arrayIndex]->addRightVertex(newVertice);
                        }
                    }
                    continue;
                }
                // set successor lanelets
                if (!(strcmp(child.name(), "successor"))) {
                    size_t successorId = child.first_attribute().as_int();
                    for (size_t i = 0; i < n; i++) {
                        if (tempLaneletContainer[i]->getId() == successorId) {
                            tempLaneletContainer[arrayIndex]->addSuccessor(tempLaneletContainer[i]);
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
                            tempLaneletContainer[arrayIndex]->addPredecessor(tempLaneletContainer[i]);
                            break;
                        }
                    }
                    continue;
                }
                // set left adjacent lanelets
                if (!(strcmp(child.name(), "adjacentLeft"))) {
                    size_t adjacentId = child.attribute("ref").as_int();
                    DrivingDirection dir{DrivingDirection::invalid};
                    if(!(strcmp(child.attribute("drivingDir").as_string(), "same")))
                        dir = DrivingDirection::same;
                    else if(!(strcmp(child.attribute("drivingDir").as_string(), "opposite")))
                        dir = DrivingDirection::opposite;
                    for (size_t i = 0; i < n; i++) {
                        if (tempLaneletContainer[i]->getId() == adjacentId) {
                            tempLaneletContainer[arrayIndex]->setLeftAdjacent(tempLaneletContainer[i], dir);
                            break;
                        }
                    }
                    continue;
                }
                // set right adjacent lanelets
                if (!(strcmp(child.name(), "adjacentRight"))) {
                    size_t adjacentId = child.attribute("ref").as_int();
                    DrivingDirection dir{DrivingDirection::invalid};
                    if(!(strcmp(child.attribute("drivingDir").as_string(), "same")))
                        dir = DrivingDirection::same;
                    else if(!(strcmp(child.attribute("drivingDir").as_string(), "opposite")))
                        dir = DrivingDirection::opposite;
                    for (size_t i = 0; i < n; i++) {
                        if (tempLaneletContainer[i]->getId() == adjacentId) {
                            tempLaneletContainer[arrayIndex]->setRightAdjacent(tempLaneletContainer[i], dir);
                            break;
                        }
                    }
                    continue;
                }
            }
            tempLaneletContainer[arrayIndex]->createCenterVertices();
            tempLaneletContainer[arrayIndex]->constructOuterPolygon();
            arrayIndex++;
        }
    }
    return tempLaneletContainer;
}

std::vector<std::shared_ptr<TrafficSign>> CommonRoadFactory2018b::createTrafficSigns() {

    std::vector<std::shared_ptr<TrafficSign>> tempLaneletContainer{};
    return tempLaneletContainer;
}

std::vector<std::shared_ptr<TrafficLight>> CommonRoadFactory2018b::createTrafficLights() {

    std::vector<std::shared_ptr<TrafficLight>> tempLaneletContainer{};
    return tempLaneletContainer;
}

std::vector<std::shared_ptr<Intersection>> CommonRoadFactory2018b::createIntersections(const std::vector<std::shared_ptr<Lanelet>>& lanelets) {

    std::vector<std::shared_ptr<Intersection>> tempLaneletContainer{};
    return tempLaneletContainer;
}
