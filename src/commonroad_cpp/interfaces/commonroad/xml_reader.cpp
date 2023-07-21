//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <cstring>
#include <utility>

#include <pugixml.hpp>

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

#include <commonroad_cpp/interfaces/commonroad/xml_reader.h>

double XMLReader::extractTimeStepSize(const std::string &xmlFile) {
    std::unique_ptr<pugi::xml_document> doc = std::make_unique<pugi::xml_document>();
    if (!doc->load_file(xmlFile.c_str()))
        throw std::runtime_error("Couldn't load XML-File: " + xmlFile);
    return doc->child("commonRoad").attribute("timeStepSize").as_double();
}

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
    const auto factory = createCommonRoadFactory(xmlFile);
    auto obstacle = factory->createObstacles();
    std::vector<std::shared_ptr<Obstacle>> dummyEgo;
    auto cou = extractCountryFromXML(xmlFile);
    auto signs = factory->createTrafficSigns();
    auto lights = factory->createTrafficLights();
    auto lanelets = factory->createLanelets(signs, lights);
    auto inters = factory->createIntersections(lanelets);
    auto roadNetwork = std::make_shared<RoadNetwork>(lanelets, cou, signs, lights, inters);
    for (const auto &inter : roadNetwork->getIntersections())
        inter->computeMemberLanelets(roadNetwork);
    double timeStepSize{factory->getTimeStepSize()};
    return std::make_shared<World>(0, roadNetwork, dummyEgo, obstacle, timeStepSize);
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

std::shared_ptr<SignalState> XMLReader::extractSignalState(const pugi::xml_node &child) {
    pugi::xml_node states = child;
    SignalState initialSignalState;
    initialSignalState.setTimeStep(states.child("time").child("exact").text().as_ullong());
    if (states.child("horn") != nullptr)
        initialSignalState.setHorn(states.child("horn").text().as_bool());
    if (states.child("indicatorLeft") != nullptr)
        initialSignalState.setIndicatorLeft(states.child("indicatorLeft").text().as_bool());
    if (states.child("indicatorRight") != nullptr)
        initialSignalState.setIndicatorRight(states.child("indicatorRight").text().as_bool());
    if (states.child("brakingLights") != nullptr)
        initialSignalState.setBrakingLights(states.child("brakingLights").text().as_bool());
    if (states.child("hazardWarningLights") != nullptr)
        initialSignalState.setHazardWarningLights(states.child("hazardWarningLights").text().as_bool());
    if (states.child("flashingBlueLights") != nullptr)
        initialSignalState.setFlashingBlueLights(states.child("flashingBlueLights").text().as_bool());
    return std::make_shared<SignalState>(initialSignalState);
}

std::shared_ptr<State> XMLReader::extractState(const pugi::xml_node &states) {
    State sta;
    sta.setTimeStep(states.child("time").child("exact").text().as_ullong());
    sta.setXPosition(states.child("position").child("point").child("x").text().as_double());
    sta.setYPosition(states.child("position").child("point").child("y").text().as_double());
    sta.setGlobalOrientation(states.child("orientation").child("exact").text().as_double());
    sta.setVelocity(states.child("velocity").child("exact").text().as_double());
    if (states.child("acceleration").child("exact").text() != nullptr)
        sta.setAcceleration(states.child("acceleration").child("exact").text().as_double());
    return std::make_shared<State>(sta);
}

void XMLReader::extractShape(const std::shared_ptr<Obstacle> &obstacle, pugi::xml_node child) {
    if ((strcmp(child.first_child().name(), "rectangle")) == 0) { // TODO: other shape types
        obstacle->setRectangleShape(child.first_child().child("length").text().as_double(),
                                    child.first_child().child("width").text().as_double());
    } else if ((strcmp(child.first_child().name(), "circle")) == 0)
        obstacle->setCircleShape(child.first_child().child("radius").text().as_double(),
                                 {child.first_child().child("center").child("x").text().as_double(),
                                  child.first_child().child("center").child("y").text().as_double()});
}

void XMLReader::createDynamicObstacle(std::vector<std::shared_ptr<Obstacle>> &obstacleList,
                                      const pugi::xml_node &roadElements) {
    std::shared_ptr<Obstacle> tempObstacle = std::make_shared<Obstacle>();

    tempObstacle->setActuatorParameters(ActuatorParameters::vehicleDefaults());
    tempObstacle->setSensorParameters(SensorParameters::dynamicDefaults());
    tempObstacle->setObstacleRole(ObstacleRole::DYNAMIC);

    // extract ID, type, shape, initial state, and trajectory
    tempObstacle->setId(roadElements.first_attribute().as_ullong());
    tempObstacle->setObstacleType(
        obstacle_operations::matchStringToObstacleType(std::next(roadElements.begin())->text().as_string()));
    for (pugi::xml_node child = roadElements.first_child(); child != nullptr; child = child.next_sibling()) {
        if ((strcmp(child.name(), "shape")) == 0) {
            extractShape(tempObstacle, child);
            continue;
        }
        if ((strcmp(child.name(), "initialState")) == 0) {
            std::shared_ptr<State> initialState{XMLReader::extractInitialState(child)};
            tempObstacle->setCurrentState(initialState);
        } else if ((strcmp(child.name(), "initialSignalState")) == 0) {
            std::shared_ptr<SignalState> initialSignalState{XMLReader::extractSignalState(child)};
            tempObstacle->setCurrentSignalState(initialSignalState);
        } else if ((strcmp(child.name(), "trajectory")) == 0) {
            for (pugi::xml_node states = child.first_child(); states != nullptr; states = states.next_sibling()) {
                tempObstacle->appendStateToTrajectoryPrediction(XMLReader::extractState(states));
            }
        } else if ((strcmp(child.name(), "signalSeries")) == 0) {
            for (pugi::xml_node states = child.first_child(); states != nullptr; states = states.next_sibling()) {
                tempObstacle->appendSignalStateToSeries(XMLReader::extractSignalState(states));
            }
        }
    }

    obstacleList.emplace_back(tempObstacle);
}

void XMLReader::extractStaticObstacle(std::vector<std::shared_ptr<Obstacle>> &obstacleList,
                                      const pugi::xml_node &roadElements) {
    std::shared_ptr<Obstacle> tempObstacle = std::make_shared<Obstacle>();
    tempObstacle->setObstacleRole(ObstacleRole::STATIC);
    tempObstacle->setSensorParameters(SensorParameters::staticDefaults());

    // extract ID, type, shape, and initial state
    tempObstacle->setId(roadElements.first_attribute().as_ullong());
    tempObstacle->setIsStatic(true);
    tempObstacle->setObstacleType(
        obstacle_operations::matchStringToObstacleType(roadElements.first_child().text().as_string()));
    for (pugi::xml_node child = roadElements.first_child(); child != nullptr; child = child.next_sibling()) {
        if ((strcmp(child.name(), "shape")) == 0) {
            extractShape(tempObstacle, child);
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
    size_t numLanelets{static_cast<size_t>(
        std::distance(commonRoad.children("lanelet").begin(), commonRoad.children("lanelet").end()))};
    tempLaneletContainer.clear();
    tempLaneletContainer.reserve(numLanelets); // Already know the size --> Faster memory allocation

    // all lanelets must be initialized first because they are referencing each other
    for (size_t i{0}; i < numLanelets; i++) {
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
    return numLanelets;
}

void XMLReader::extractLaneletBoundary(const std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer,
                                       size_t arrayIndex, const pugi::xml_node &child, const char *side) {
    for (pugi::xml_node points = child.first_child(); points != nullptr; points = points.next_sibling()) {
        vertex newVertex;
        if ((strcmp(points.name(), "point")) == 0) {
            newVertex = {points.child("x").text().as_double(), points.child("y").text().as_double()};
            if ((strcmp(side, "rightBound")) == 0)
                tempLaneletContainer[arrayIndex]->addRightVertex(newVertex);
            else if ((strcmp(side, "leftBound")) == 0)
                tempLaneletContainer[arrayIndex]->addLeftVertex(newVertex);
        }
        if ((strcmp(points.name(), "lineMarking")) == 0) {
            auto lineMarking = lanelet_operations::matchStringToLineMarking(points.first_child().value());
            if ((strcmp(side, "rightBound")) == 0)
                tempLaneletContainer[arrayIndex]->setLineMarkingRight(lineMarking);
            else if ((strcmp(side, "leftBound")) == 0)
                tempLaneletContainer[arrayIndex]->setLineMarkingLeft(lineMarking);
        }
    }
}

void XMLReader::extractLaneletPreSuc(const std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer,
                                     size_t arrayIndex, const pugi::xml_node &child, const char *type) {
    size_t lid{child.first_attribute().as_ullong()};
    for (size_t i{0}; i < tempLaneletContainer.size(); i++) {
        if (tempLaneletContainer[i]->getId() == lid) {
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
    bool oppositeDir{false};
    if ((strcmp(child.attribute("drivingDir").as_string(), "same")) == 0)
        oppositeDir = false;
    else if ((strcmp(child.attribute("drivingDir").as_string(), "opposite")) == 0)
        oppositeDir = true;
    for (size_t i{0}; i < tempLaneletContainer.size(); i++) {
        if (tempLaneletContainer[i]->getId() == adjacentId) {
            if ((strcmp(type, "adjacentLeft")) == 0)
                tempLaneletContainer[arrayIndex]->setLeftAdjacent(tempLaneletContainer[i], oppositeDir);
            else if ((strcmp(type, "adjacentRight")) == 0)
                tempLaneletContainer[arrayIndex]->setRightAdjacent(tempLaneletContainer[i], oppositeDir);
            break;
        }
    }
}
