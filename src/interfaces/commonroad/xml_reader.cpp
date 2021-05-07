//
// Created by Sebastian Maierhofer on 30.10.20.
//
#include <utility>

#include "../../obstacle/obstacle_operations.h"
#include "../../roadNetwork/lanelet/lanelet_operations.h"
#include "commonroad_factory_2018b.h"
#include "commonroad_factory_2020a.h"
#include "xml_reader.h"

std::unique_ptr<CommonRoadFactory> createCommonRoadFactory(const std::string &xmlFile) {
    std::unique_ptr<pugi::xml_document> doc = std::make_unique<pugi::xml_document>();

    if (!doc->load_file(xmlFile.c_str()))
        throw std::runtime_error("Couldn't load XML-File");

    const auto version = doc->child("commonRoad").attribute("commonRoadVersion").value();
    if (!strcmp(version, "2017a") || !strcmp(version, "2018b"))
        return std::make_unique<CommonRoadFactory2018b>(std::move(doc));
    else if (!strcmp(version, "2020a"))
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

std::vector<std::shared_ptr<TrafficSign>> XMLReader::createTrafficSignFromXML(const std::string &xmlFile) {
    const auto factory = createCommonRoadFactory(xmlFile);
    return factory->createTrafficSigns();
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
    initialState.setTimeStep(states.child("time").child("exact").text().as_int());
    initialState.setXPosition(states.child("position").child("point").child("x").text().as_double());
    initialState.setYPosition(states.child("position").child("point").child("y").text().as_double());
    initialState.setGlobalOrientation(states.child("orientation").child("exact").text().as_double());
    initialState.setVelocity(states.child("velocity").child("exact").text().as_double());
    initialState.setAcceleration(states.child("acceleration").child("exact").text().as_double());
    return std::make_shared<State>(initialState);
}

std::shared_ptr<State> XMLReader::extractState(const pugi::xml_node &states) {
    State st;
    st.setTimeStep(states.child("time").child("exact").text().as_int());
    st.setXPosition(states.child("position").child("point").child("x").text().as_double());
    st.setYPosition(states.child("position").child("point").child("y").text().as_double());
    st.setGlobalOrientation(states.child("orientation").child("exact").text().as_double());
    st.setVelocity(states.child("velocity").child("exact").text().as_double());
    st.setAcceleration(states.child("acceleration").child("exact").text().as_double());
    return std::make_shared<State>(st);
}

void XMLReader::createDynamicObstacle(std::vector<std::shared_ptr<Obstacle>> &obstacleList,
                                      const pugi::xml_node &roadElements) {
    std::shared_ptr<Obstacle> tempObstacle = std::make_shared<Obstacle>();

    // extract ID, type, shape, initial state, and trajectory
    tempObstacle->setId(roadElements.first_attribute().as_int());
    tempObstacle->setObstacleType(matchStringToObstacleType(roadElements.first_child().text().as_string()));
    for (pugi::xml_node child = roadElements.first_child(); child; child = child.next_sibling()) {
        if (!(strcmp(child.name(), "shape"))) { // TODO: other shape types
            if (!(strcmp(child.first_child().name(), "rectangle"))) {
                tempObstacle->getGeoShape().setLength(child.first_child().child("length").text().as_double());
                tempObstacle->getGeoShape().setWidth(child.first_child().child("width").text().as_double());
            }
            continue;
        }
        if (!(strcmp(child.name(), "initialState"))) {
            std::shared_ptr<State> initialState{XMLReader::extractInitialState(child)};
            tempObstacle->setCurrentState(initialState);
            tempObstacle->appendStateToTrajectoryPrediction(initialState);
        } else if (!(strcmp(child.name(), "trajectory"))) {
            for (pugi::xml_node states = child.first_child(); states; states = states.next_sibling()) {
                tempObstacle->appendStateToTrajectoryPrediction(XMLReader::extractState(states));
            }
        }
    }
    tempObstacle->setReactionTime(reactionTimeObstacles);
    obstacleList.emplace_back(tempObstacle);
}

void XMLReader::extractStaticObstacle(std::vector<std::shared_ptr<Obstacle>> &obstacleList,
                                      const pugi::xml_node &roadElements) {
    std::shared_ptr<Obstacle> tempObstacle = std::make_shared<Obstacle>();

    // extract ID, type, shape, and initial state
    tempObstacle->setId(roadElements.first_attribute().as_int());
    tempObstacle->setIsStatic(true);
    tempObstacle->setObstacleType(matchStringToObstacleType(roadElements.first_child().text().as_string()));
    for (pugi::xml_node child = roadElements.first_child(); child; child = child.next_sibling()) {
        if (!(strcmp(child.name(), "shape"))) {
            if (!(strcmp(child.first_child().name(), "rectangle"))) { // TODO: other shape types
                tempObstacle->getGeoShape().setLength(child.first_child().child("length").text().as_double());
                tempObstacle->getGeoShape().setWidth(child.first_child().child("width").text().as_double());
            }
            continue;
        } else if (!(strcmp(child.name(), "initialState"))) {
            std::shared_ptr<State> initialState{XMLReader::extractInitialState(child)};
            tempObstacle->setCurrentState(initialState);
        }
    }
    obstacleList.emplace_back(tempObstacle);
}

int XMLReader::initializeLanelets(std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer,
                                  const pugi::xml_node &commonRoad) {
    // get the number of lanelets
    int n = std::distance(commonRoad.children("lanelet").begin(), commonRoad.children("lanelet").end());
    tempLaneletContainer.clear();
    tempLaneletContainer.reserve(n); // Already know the size --> Faster memory allocation

    // all lanelets must be initialized first because they are referencing each other
    for (int i = 0; i < n; i++) {
        std::shared_ptr<Lanelet> tempLanelet =
            std::make_shared<Lanelet>(); // make_shared is faster than (new Lanelet());
        tempLaneletContainer.emplace_back(tempLanelet);
    }

    int arrayIndex = 0;
    // set id of lanelets
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements;
         roadElements = roadElements.next_sibling()) {
        if (!(strcmp(roadElements.name(), "lanelet"))) {
            tempLaneletContainer[arrayIndex]->setId(roadElements.first_attribute().as_int());
            arrayIndex++;
        }
    }
    return n;
}

void XMLReader::extractLaneletBoundary(const std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer,
                                       int arrayIndex, const pugi::xml_node &child, const char *side) {
    for (pugi::xml_node points = child.first_child(); points; points = points.next_sibling()) {
        vertex newVertex{};
        LineMarking lineMarking;
        if (!(strcmp(points.name(), "point"))) {
            newVertex = {points.child("x").text().as_double(), points.child("y").text().as_double()};
            if (!(strcmp(side, "rightBound")))
                tempLaneletContainer[arrayIndex]->addRightVertex(newVertex);
            else if (!(strcmp(side, "leftBound")))
                tempLaneletContainer[arrayIndex]->addLeftVertex(newVertex);
        }
        if (!(strcmp(points.name(), "lineMarking"))) {
            lineMarking = matchStringToLineMarking(points.value());
            if (!(strcmp(side, "rightBound")))
                tempLaneletContainer[arrayIndex]->setLineMarkingRight(lineMarking);
            else if (!(strcmp(side, "leftBound")))
                tempLaneletContainer[arrayIndex]->setLineMarkingLeft(lineMarking);
        }
    }
}

void XMLReader::extractLaneletPreSuc(const std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer, int arrayIndex,
                                     const pugi::xml_node &child, const char *type) {
    int id = child.first_attribute().as_int();
    for (int i = 0; i < tempLaneletContainer.size(); i++) {
        if (tempLaneletContainer[i]->getId() == id) {
            if (!(strcmp(type, "successor")))
                tempLaneletContainer[arrayIndex]->addSuccessor(tempLaneletContainer[i]);
            else if (!(strcmp(type, "predecessor")))
                tempLaneletContainer[arrayIndex]->addPredecessor(tempLaneletContainer[i]);
            break;
        }
    }
}

void XMLReader::extractLaneletAdjacency(const std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer,
                                        int arrayIndex, const pugi::xml_node &child, const char *type) {
    int adjacentId = child.attribute("ref").as_int();
    DrivingDirection dir{DrivingDirection::invalid};
    if (!(strcmp(child.attribute("drivingDir").as_string(), "same")))
        dir = DrivingDirection::same;
    else if (!(strcmp(child.attribute("drivingDir").as_string(), "opposite")))
        dir = DrivingDirection::opposite;
    for (int i = 0; i < tempLaneletContainer.size(); i++) {
        if (tempLaneletContainer[i]->getId() == adjacentId) {
            if (!(strcmp(type, "adjacentLeft")))
                tempLaneletContainer[arrayIndex]->setLeftAdjacent(tempLaneletContainer[i], dir);
            else if (!(strcmp(type, "adjacentRight")))
                tempLaneletContainer[arrayIndex]->setRightAdjacent(tempLaneletContainer[i], dir);
            break;
        }
    }
}
