//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <cstring>
#include <sstream>
#include <utility>

#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include "commonroad_cpp/roadNetwork/intersection/intersection.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign.h>

#include "commonroad_factory_2018b.h"
#include "commonroad_factory_2020a.h"

#include "xml_reader.h"

std::unique_ptr<CommonRoadFactory> createCommonRoadFactory(const std::string &xmlFile) {
    std::unique_ptr<pugi::xml_document> doc = std::make_unique<pugi::xml_document>();

    if (!doc->load_file(xmlFile.c_str()))
        throw std::runtime_error("Couldn't load XML-File: " + xmlFile);

    const auto *const version = doc->child("commonRoad").attribute("commonRoadVersion").value();
    if ((strcmp(version, "2017a") == 0) || (strcmp(version, "2018b") == 0))
        return std::make_unique<CommonRoadFactory2018b>(std::move(doc));
    else if (strcmp(version, "2020a") == 0)
        return std::make_unique<CommonRoadFactory2020a>(std::move(doc));
    else
        throw std::runtime_error("This CommonRoad version is not supported.");
}

std::vector<std::shared_ptr<Obstacle>> XMLReader::createObstacleFromXML(const std::string &xmlFile) {
    const auto factory = createCommonRoadFactory(xmlFile);
    return factory->createObstacles();
}

std::vector<std::shared_ptr<Lanelet>>
XMLReader::createLaneletFromXML(const std::string &xmlFile, std::vector<std::shared_ptr<TrafficSign>> trafficSigns,
                                std::vector<std::shared_ptr<TrafficLight>> trafficLights) {
    const auto factory = createCommonRoadFactory(xmlFile);
    return factory->createLanelets(std::move(trafficSigns), std::move(trafficLights));
}
std::shared_ptr<World> XMLReader::createWorldFromXML(const std::string &xmlFile) {
    auto obstacle = createObstacleFromXML(xmlFile);
    auto network = createLaneletFromXML(xmlFile);
    std::vector<std::shared_ptr<Obstacle>> dummyEgo;
    auto cou = extractCountryFromXML(xmlFile);
    auto inters = createIntersectionFromXML(xmlFile, network);
    auto signs = createTrafficSignFromXML(xmlFile);
    auto lights = createTrafficLightFromXML(xmlFile);
    auto roadNetwork = std::make_shared<RoadNetwork>(network, cou, signs, lights, inters);
    return std::make_shared<World>(0, roadNetwork, dummyEgo, obstacle);
}

std::vector<std::shared_ptr<TrafficSign>> XMLReader::createTrafficSignFromXML(const std::string &xmlFile) {
    const auto factory = createCommonRoadFactory(xmlFile);
    return factory->createTrafficSigns();
}

SupportedTrafficSignCountry XMLReader::extractCountryFromXML(const std::string &xmlFile) {
    std::vector<std::string> result;
    std::stringstream stream{xmlFile}; // create string stream from the string
    while (stream.good()) {
        std::string substr;
        getline(stream, substr, '/'); // get first string delimited by comma
        result.push_back(substr);
    }
    auto name{result.back().substr(0, 3)};
    return RoadNetwork::matchStringToCountry(name);
}

std::vector<std::shared_ptr<TrafficLight>> XMLReader::createTrafficLightFromXML(const std::string &xmlFile) {
    const auto factory = createCommonRoadFactory(xmlFile);
    return factory->createTrafficLights();
}

std::vector<std::shared_ptr<Intersection>>
XMLReader::createIntersectionFromXML(const std::string &xmlFile,
                                     const std::vector<std::shared_ptr<Lanelet>> &lanelets) {
    const auto factory = createCommonRoadFactory(xmlFile);
    return factory->createIntersections(lanelets);
}

std::shared_ptr<State> XMLReader::extractInitialState(const pugi::xml_node &child) {
    pugi::xml_node states = child;
    State initialState;
    initialState.setTimeStep(states.child("time").child("exact").text().as_ullong());
    initialState.setXPosition(states.child("position").child("point").child("x").text().as_double());
    initialState.setYPosition(states.child("position").child("point").child("y").text().as_double());
    initialState.setGlobalOrientation(states.child("orientation").child("exact").text().as_double());
    if (states.child("velocity").child("exact").text() != nullptr)
        initialState.setVelocity(states.child("velocity").child("exact").text().as_double());
    if (states.child("acceleration").child("exact").text() != nullptr)
        initialState.setAcceleration(states.child("acceleration").child("exact").text().as_double());
    return std::make_shared<State>(initialState);
}

std::shared_ptr<State> XMLReader::extractState(const pugi::xml_node &states) {
    State st;
    st.setTimeStep(states.child("time").child("exact").text().as_ullong());
    st.setXPosition(states.child("position").child("point").child("x").text().as_double());
    st.setYPosition(states.child("position").child("point").child("y").text().as_double());
    st.setGlobalOrientation(states.child("orientation").child("exact").text().as_double());
    st.setVelocity(states.child("velocity").child("exact").text().as_double());
    if (states.child("acceleration").child("exact").text() != nullptr)
        st.setAcceleration(states.child("acceleration").child("exact").text().as_double());
    return std::make_shared<State>(st);
}

void XMLReader::createDynamicObstacle(std::vector<std::shared_ptr<Obstacle>> &obstacleList,
                                      const pugi::xml_node &roadElements) {
    std::shared_ptr<Obstacle> tempObstacle = std::make_shared<Obstacle>();

    // extract ID, type, shape, initial state, and trajectory
    tempObstacle->setId(roadElements.first_attribute().as_ullong());
    tempObstacle->setObstacleType(
        obstacle_operations::matchStringToObstacleType(roadElements.first_child().text().as_string()));
    for (pugi::xml_node child = roadElements.first_child(); child != nullptr; child = child.next_sibling()) {
        if ((strcmp(child.name(), "shape")) == 0) { // TODO: other shape types
            if ((strcmp(child.first_child().name(), "rectangle")) == 0) {
                auto &geoRectangle = dynamic_cast<Rectangle &>(tempObstacle->getGeoShape());
                geoRectangle.setLength(child.first_child().child("length").text().as_double());
                geoRectangle.setWidth(child.first_child().child("width").text().as_double());
            }
            continue;
        }
        if ((strcmp(child.name(), "initialState")) == 0) {
            std::shared_ptr<State> initialState{XMLReader::extractInitialState(child)};
            tempObstacle->setCurrentState(initialState);
        } else if ((strcmp(child.name(), "trajectory")) == 0) {
            for (pugi::xml_node states = child.first_child(); states != nullptr; states = states.next_sibling()) {
                tempObstacle->appendStateToTrajectoryPrediction(XMLReader::extractState(states));
            }
        }
    }
    obstacleList.emplace_back(tempObstacle);
}

void XMLReader::extractStaticObstacle(std::vector<std::shared_ptr<Obstacle>> &obstacleList,
                                      const pugi::xml_node &roadElements) {
    std::shared_ptr<Obstacle> tempObstacle = std::make_shared<Obstacle>();

    // extract ID, type, shape, and initial state
    tempObstacle->setId(roadElements.first_attribute().as_ullong());
    tempObstacle->setIsStatic(true);
    tempObstacle->setObstacleType(
        obstacle_operations::matchStringToObstacleType(roadElements.first_child().text().as_string()));
    for (pugi::xml_node child = roadElements.first_child(); child != nullptr; child = child.next_sibling()) {
        if ((strcmp(child.name(), "shape")) == 0) {
            if ((strcmp(child.first_child().name(), "rectangle")) == 0) { // TODO: other shape types
                auto &geoRectangle = dynamic_cast<Rectangle &>(tempObstacle->getGeoShape());
                geoRectangle.setLength(child.first_child().child("length").text().as_double());
                geoRectangle.setWidth(child.first_child().child("width").text().as_double());
            }
            continue;
        } else if ((strcmp(child.name(), "initialState")) == 0) {
            std::shared_ptr<State> initialState{XMLReader::extractInitialState(child)};
            tempObstacle->setCurrentState(initialState);
        }
    }
    obstacleList.emplace_back(tempObstacle);
}

size_t XMLReader::initializeLanelets(std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer,
                                     const pugi::xml_node &commonRoad) {
    // get the number of lanelets
    size_t n{static_cast<size_t>(
        std::distance(commonRoad.children("lanelet").begin(), commonRoad.children("lanelet").end()))};
    tempLaneletContainer.clear();
    tempLaneletContainer.reserve(n); // Already know the size --> Faster memory allocation

    // all lanelets must be initialized first because they are referencing each other
    for (size_t i{0}; i < n; i++) {
        std::shared_ptr<Lanelet> tempLanelet =
            std::make_shared<Lanelet>(); // make_shared is faster than (new Lanelet());
        tempLaneletContainer.emplace_back(tempLanelet);
    }

    int arrayIndex = 0;
    // set id of lanelets
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements != nullptr;
         roadElements = roadElements.next_sibling()) {
        if ((strcmp(roadElements.name(), "lanelet")) == 0) {
            tempLaneletContainer[static_cast<unsigned long>(arrayIndex)]->setId(
                static_cast<size_t>(roadElements.first_attribute().as_int()));
            arrayIndex++;
        }
    }
    return n;
}

void XMLReader::extractLaneletBoundary(const std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer,
                                       size_t arrayIndex, const pugi::xml_node &child, const char *side) {
    for (pugi::xml_node points = child.first_child(); points != nullptr; points = points.next_sibling()) {
        vertex newVertex;
        LineMarking lineMarking;
        if ((strcmp(points.name(), "point")) == 0) {
            newVertex = {points.child("x").text().as_double(), points.child("y").text().as_double()};
            if ((strcmp(side, "rightBound")) == 0)
                tempLaneletContainer[arrayIndex]->addRightVertex(newVertex);
            else if ((strcmp(side, "leftBound")) == 0)
                tempLaneletContainer[arrayIndex]->addLeftVertex(newVertex);
        }
        if ((strcmp(points.name(), "lineMarking")) == 0) {
            lineMarking = lanelet_operations::matchStringToLineMarking(points.value());
            if ((strcmp(side, "rightBound")) == 0)
                tempLaneletContainer[arrayIndex]->setLineMarkingRight(lineMarking);
            else if ((strcmp(side, "leftBound")) == 0)
                tempLaneletContainer[arrayIndex]->setLineMarkingLeft(lineMarking);
        }
    }
}

void XMLReader::extractLaneletPreSuc(const std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer,
                                     size_t arrayIndex, const pugi::xml_node &child, const char *type) {
    size_t id{child.first_attribute().as_ullong()};
    for (size_t i{0}; i < tempLaneletContainer.size(); i++) {
        if (tempLaneletContainer[i]->getId() == id) {
            if ((strcmp(type, "successor")) == 0)
                tempLaneletContainer[arrayIndex]->addSuccessor(tempLaneletContainer[i]);
            else if ((strcmp(type, "predecessor")) == 0)
                tempLaneletContainer[arrayIndex]->addPredecessor(tempLaneletContainer[i]);
            break;
        }
    }
}

void XMLReader::extractLaneletAdjacency(const std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer,
                                        size_t arrayIndex, const pugi::xml_node &child, const char *type) {
    size_t adjacentId{child.attribute("ref").as_ullong()};
    DrivingDirection dir{DrivingDirection::invalid};
    if ((strcmp(child.attribute("drivingDir").as_string(), "same")) == 0)
        dir = DrivingDirection::same;
    else if ((strcmp(child.attribute("drivingDir").as_string(), "opposite")) == 0)
        dir = DrivingDirection::opposite;
    for (size_t i{0}; i < tempLaneletContainer.size(); i++) {
        if (tempLaneletContainer[i]->getId() == adjacentId) {
            if ((strcmp(type, "adjacentLeft")) == 0)
                tempLaneletContainer[arrayIndex]->setLeftAdjacent(tempLaneletContainer[i], dir);
            else if ((strcmp(type, "adjacentRight")) == 0)
                tempLaneletContainer[arrayIndex]->setRightAdjacent(tempLaneletContainer[i], dir);
            break;
        }
    }
}
