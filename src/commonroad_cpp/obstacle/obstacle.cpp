//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "obstacle.h"
#include "../geometry/geometric_operations.h"
#include "../roadNetwork/lanelet/lanelet_operations.h"

#include <algorithm> // for max, min
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <cmath>
#include <stdexcept> // for logic_error
#include <string>    // for operator+
#include <utility>

#include <Eigen/Core> // for Vector2d

#include <boost/geometry/geometries/ring.hpp> // for ring

#include <commonroad_cpp/auxiliaryDefs/structs.h>
#include <commonroad_cpp/auxiliaryDefs/types_and_definitions.h>
#include <commonroad_cpp/geometry/rectangle.h>
#include <commonroad_cpp/geometry/shape.h>
#include <commonroad_cpp/obstacle/state.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/road_network.h>

Obstacle::Obstacle(size_t obstacleId, bool isStatic, std::shared_ptr<State> currentState, ObstacleType obstacleType,
                   double vMax, double aMax, double aMaxLong, double aMinLong, double reactionTime,
                   std::map<size_t, std::shared_ptr<State>> trajectoryPrediction, double length, double width,
                   std::vector<vertex> route)
    : obstacleId(obstacleId), isStatic(isStatic), currentState(std::move(currentState)), obstacleType(obstacleType),
      vMax(vMax), aMax(aMax), aMaxLong(aMaxLong), aMinLong(aMinLong), reactionTime(reactionTime),
      trajectoryPrediction(std::move(trajectoryPrediction)), geoShape(Rectangle(length, width)), route(route) {
    if (isStatic)
        setIsStatic(isStatic);

    if (route.empty()) {
        route.reserve(trajectoryPrediction.size());
        size_t idx{0};
        for (const auto &state : trajectoryPrediction)
            route[idx] = {state.second->getXPosition(), state.second->getYPosition()};
    }
}

void Obstacle::setId(const size_t oId) { obstacleId = oId; }

void Obstacle::setIsStatic(bool staticObstacle) {
    isStatic = staticObstacle;
    if (staticObstacle) {
        vMax = 0.0;
        aMax = 0.0;
        aMinLong = 0.0;
        aMaxLong = 0.0;
    }
}

void Obstacle::setCurrentState(const std::shared_ptr<State> &state) { currentState = state; }

void Obstacle::setObstacleType(ObstacleType type) { obstacleType = type; }

void Obstacle::setVmax(const double vmax) { vMax = isStatic ? 0.0 : vmax; }

void Obstacle::setAmax(const double amax) { aMax = isStatic ? 0.0 : amax; }

void Obstacle::setAmaxLong(const double amax) { aMaxLong = isStatic ? 0.0 : amax; }

void Obstacle::setAminLong(const double amin) { aMinLong = isStatic ? 0.0 : amin; }

void Obstacle::setReactionTime(const double tReact) { reactionTime = isStatic ? 0.0 : tReact; }

void Obstacle::setTrajectoryPrediction(const std::map<size_t, std::shared_ptr<State>> &trajPrediction) {
    trajectoryPrediction = trajPrediction;
}

void Obstacle::setRectangleShape(double length, double width) { geoShape = Rectangle(length, width); }

void Obstacle::appendStateToTrajectoryPrediction(const std::shared_ptr<State> &state) {
    trajectoryPrediction.insert(std::pair<size_t, std::shared_ptr<State>>(state->getTimeStep(), state));
}

void Obstacle::appendStateToHistory(const std::shared_ptr<State> &state) {
    history.insert(std::pair<size_t, std::shared_ptr<State>>(state->getTimeStep(), state));
}

size_t Obstacle::getId() const { return obstacleId; }

bool Obstacle::getIsStatic() const { return isStatic; }

const std::shared_ptr<State> &Obstacle::getCurrentState() const { return currentState; }

bool Obstacle::timeStepExists(size_t timeStep) {
    return (isStatic or trajectoryPrediction.count(timeStep) == 1 or currentState->getTimeStep() == timeStep or
            history.count(timeStep) == 1);
}

std::shared_ptr<State> Obstacle::getStateByTimeStep(size_t timeStep) const {
    if (isStatic)
        return currentState;
    else if (trajectoryPrediction.count(timeStep) == 1)
        return trajectoryPrediction.at(timeStep);
    else if (currentState->getTimeStep() == timeStep)
        return currentState;
    else if (history.count(timeStep) == 1)
        return history.at(timeStep);
    else
        throw std::logic_error("Time step does not exist. Obstacle ID: " + std::to_string(this->getId()) +
                               " - Time step: " + std::to_string(timeStep));
}

ObstacleType Obstacle::getObstacleType() const { return obstacleType; }

double Obstacle::getVmax() const { return vMax; }

double Obstacle::getAmax() const { return aMax; }

double Obstacle::getAmaxLong() const { return aMaxLong; }

double Obstacle::getAminLong() const { return aMinLong; }

double Obstacle::getReactionTime() const { return reactionTime; }

std::vector<size_t> Obstacle::getPredictionTimeSteps() {
    std::vector<size_t> timeSteps;
    boost::copy(trajectoryPrediction | boost::adaptors::map_keys, std::back_inserter(timeSteps));
    return timeSteps;
}

std::vector<size_t> Obstacle::getHistoryTimeSteps() {
    std::vector<size_t> timeSteps;
    boost::copy(history | boost::adaptors::map_keys, std::back_inserter(timeSteps));
    return timeSteps;
}

std::vector<size_t> Obstacle::getTimeSteps() {
    std::vector<size_t> timeSteps{getPredictionTimeSteps()};
    if (timeSteps.front() != currentState->getTimeStep())
        timeSteps.insert(timeSteps.begin(), currentState->getTimeStep());
    return timeSteps;
}

std::map<size_t, std::shared_ptr<State>> Obstacle::getTrajectoryPrediction() const { return trajectoryPrediction; }

size_t Obstacle::getTrajectoryLength() const { return trajectoryPrediction.size(); }

polygon_type Obstacle::getOccupancyPolygonShape(size_t timeStep) { return setOccupancyPolygonShape(timeStep); }

polygon_type Obstacle::setOccupancyPolygonShape(size_t timeStep) {
    if (shapeAtTimeStep.count(timeStep) == 1)
        return shapeAtTimeStep[timeStep];
    std::vector<vertex> boundingRectangleVertices;
    polygon_type polygonShape;
    std::shared_ptr<State> state{this->getStateByTimeStep(timeStep)};

    if (this->getGeoShape().getType() == ShapeType::rectangle) {
        // p are vertices of the bounding rectangle
        // vertices p represent the occupancy with vehicle dimensions (Theorem 1 in SPOT paper)
        boundingRectangleVertices = geometric_operations::addObjectDimensions(
            std::vector<vertex>{vertex{0.0, 0.0}}, this->getGeoShape().getLength(), this->getGeoShape().getWidth());

        /*
         * rotate and translate the vertices of the occupancy set in local
         * coordinates to the object's reference position and rotation
         */
        std::vector<vertex> adjustedBoundingRectangleVertices = geometric_operations::rotateAndTranslateVertices(
            boundingRectangleVertices, vertex{state->getXPosition(), state->getYPosition()},
            state->getGlobalOrientation());

        polygonShape.outer().resize(adjustedBoundingRectangleVertices.size() + 1);

        // make polygon shape from previously created vertices
        for (size_t i{0}; i < adjustedBoundingRectangleVertices.size(); i++) {
            polygonShape.outer()[i] =
                point_type{adjustedBoundingRectangleVertices[i].x, adjustedBoundingRectangleVertices[i].y};
        }

        // add first point once again at the end
        if (!adjustedBoundingRectangleVertices.empty()) {
            polygonShape.outer().back() =
                point_type{adjustedBoundingRectangleVertices[0].x, adjustedBoundingRectangleVertices[0].y};
        }
    }
    shapeAtTimeStep[timeStep] = polygonShape;
    return polygonShape;
}

Shape &Obstacle::getGeoShape() { return geoShape; }

std::vector<std::shared_ptr<Lanelet>>
Obstacle::setOccupiedLaneletsByShape(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    if (occupiedLanelets.find(timeStep) != occupiedLanelets.end())
        return occupiedLanelets.at(timeStep);
    polygon_type polygonShape{getOccupancyPolygonShape(timeStep)};
    std::vector<std::shared_ptr<Lanelet>> occupied{roadNetwork->findOccupiedLaneletsByShape(polygonShape)};
    occupiedLanelets.insert(std::pair<int, std::vector<std::shared_ptr<Lanelet>>>(timeStep, occupied));
    return occupied;
}

std::vector<std::shared_ptr<Lanelet>>
Obstacle::getOccupiedLaneletsByShape(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    return setOccupiedLaneletsByShape(roadNetwork, timeStep);
    ;
}

double Obstacle::frontS(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    double lonPosition = getLonPosition(roadNetwork, timeStep);
    double theta = getCurvilinearOrientation(roadNetwork, timeStep);
    double width = geoShape.getWidth();
    double length = geoShape.getLength();

    // use maximum of all corners
    return std::max({(length / 2) * cos(theta) - (width / 2) * sin(theta) + lonPosition,
                     (length / 2) * cos(theta) - (-width / 2) * sin(theta) + lonPosition,
                     (-length / 2) * cos(theta) - (width / 2) * sin(theta) + lonPosition,
                     (-length / 2) * cos(theta) - (-width / 2) * sin(theta) + lonPosition});
}

void Obstacle::convertPointToCurvilinear(size_t timeStep, const std::shared_ptr<Lane> &refLane) {
    Eigen::Vector2d convertedPoint;
    convertedPoint = refLane->getCurvilinearCoordinateSystem().convertToCurvilinearCoords(
        getStateByTimeStep(timeStep)->getXPosition(), getStateByTimeStep(timeStep)->getYPosition());
    double theta = geometric_operations::subtractOrientations(
        getStateByTimeStep(timeStep)->getGlobalOrientation(),
        refLane->getOrientationAtPosition(getStateByTimeStep(timeStep)->getXPosition(),
                                          getStateByTimeStep(timeStep)->getYPosition()));
    convertedPositions[timeStep][refLane->getContainedLaneletIDs()] = {convertedPoint.x(), convertedPoint.y(), theta};
}

double Obstacle::frontS(size_t timeStep, const std::shared_ptr<Lane> &refLane) {
    if (!(convertedPositions.count(timeStep) == 1 and
          convertedPositions[timeStep].count(refLane->getContainedLaneletIDs()) == 1)) {
        try {
            convertPointToCurvilinear(timeStep, refLane);
        } catch (...) {
            std::string refInfo;
            for (const auto &ref : refLane->getCurvilinearCoordinateSystem().referencePath())
                refInfo += "{" + std::to_string(ref.x()) + ", " + std::to_string(ref.y()) + "}, ";
            throw std::runtime_error("Obstacle::frontS Custom CCS - Curvilinear Projection Error - Obstacle ID: " +
                                     std::to_string(obstacleId) + " - Time Step: " + std::to_string(timeStep) +
                                     " - Reference Lane: " + refInfo);
        }
    }
    double lonPosition = convertedPositions[timeStep][refLane->getContainedLaneletIDs()][0];
    double theta = convertedPositions[timeStep][refLane->getContainedLaneletIDs()][2];
    double width = geoShape.getWidth();
    double length = geoShape.getLength();

    // use maximum of all corners
    return std::max({(length / 2) * cos(theta) - (width / 2) * sin(theta) + lonPosition,
                     (length / 2) * cos(theta) - (-width / 2) * sin(theta) + lonPosition,
                     (-length / 2) * cos(theta) - (width / 2) * sin(theta) + lonPosition,
                     (-length / 2) * cos(theta) - (-width / 2) * sin(theta) + lonPosition});
}

double Obstacle::rearS(size_t timeStep, const std::shared_ptr<Lane> &refLane) {
    if (!(convertedPositions.count(timeStep) == 1 and
          convertedPositions[timeStep].count(refLane->getContainedLaneletIDs()) == 1)) {
        try {
            convertPointToCurvilinear(timeStep, refLane);
        } catch (...) {
            std::string refInfo;
            for (const auto &ref : refLane->getCurvilinearCoordinateSystem().referencePath())
                refInfo += "{" + std::to_string(ref.x()) + ", " + std::to_string(ref.y()) + "}, ";
            throw std::runtime_error("Obstacle::rearS Custom CCS - Curvilinear Projection Error - Obstacle ID: " +
                                     std::to_string(obstacleId) + " - Time Step: " + std::to_string(timeStep) +
                                     " - Reference Lane: " + refInfo);
        }
    }
    double lonPosition = convertedPositions[timeStep][refLane->getContainedLaneletIDs()][0];
    double theta = convertedPositions[timeStep][refLane->getContainedLaneletIDs()][2];
    double width = geoShape.getWidth();
    double length = geoShape.getLength();

    // use minimum of all corners
    return std::min({(length / 2) * cos(theta) - (width / 2) * sin(theta) + lonPosition,
                     (length / 2) * cos(theta) - (-width / 2) * sin(theta) + lonPosition,
                     (-length / 2) * cos(theta) - (width / 2) * sin(theta) + lonPosition,
                     (-length / 2) * cos(theta) - (-width / 2) * sin(theta) + lonPosition});
}

double Obstacle::rightD(size_t timeStep, const std::shared_ptr<Lane> &refLane) {
    if (!(convertedPositions.count(timeStep) == 1 and
          convertedPositions[timeStep].count(refLane->getContainedLaneletIDs()) == 1)) {
        try {
            convertPointToCurvilinear(timeStep, refLane);
        } catch (...) {
            std::string refInfo;
            for (const auto &ref : refLane->getCurvilinearCoordinateSystem().referencePath())
                refInfo += "{" + std::to_string(ref.x()) + ", " + std::to_string(ref.y()) + "}, ";
            throw std::runtime_error("Obstacle::rightD Custom CCS - Curvilinear Projection Error - Obstacle ID: " +
                                     std::to_string(obstacleId) + " - Time Step: " + std::to_string(timeStep) +
                                     " - Reference Lane: " + refInfo);
        }
    }
    double latPosition = convertedPositions[timeStep][refLane->getContainedLaneletIDs()][1];
    double theta = convertedPositions[timeStep][refLane->getContainedLaneletIDs()][2];
    double width = geoShape.getWidth();
    double length = geoShape.getLength();

    return std::min({(width / 2) * cos(theta) - (length / 2) * sin(theta) + latPosition,
                     (width / 2) * cos(theta) - (-length / 2) * sin(theta) + latPosition,
                     (-width / 2) * cos(theta) - (length / 2) * sin(theta) + latPosition,
                     (-width / 2) * cos(theta) - (-length / 2) * sin(theta) + latPosition});
}

double Obstacle::leftD(size_t timeStep, const std::shared_ptr<Lane> &refLane) {
    if (!(convertedPositions.count(timeStep) == 1 and
          convertedPositions[timeStep].count(refLane->getContainedLaneletIDs()) == 1)) {
        try {
            convertPointToCurvilinear(timeStep, refLane);
        } catch (...) {
            std::string refInfo;
            for (const auto &ref : refLane->getCurvilinearCoordinateSystem().referencePath())
                refInfo += "{" + std::to_string(ref.x()) + ", " + std::to_string(ref.y()) + "}, ";
            throw std::runtime_error("Obstacle::leftD Custom CCS - Curvilinear Projection Error - Obstacle ID: " +
                                     std::to_string(obstacleId) + " - Time Step: " + std::to_string(timeStep) +
                                     " - Reference Lane: " + refInfo);
        }
    }
    double latPosition = convertedPositions[timeStep][refLane->getContainedLaneletIDs()][1];
    double theta = convertedPositions[timeStep][refLane->getContainedLaneletIDs()][2];
    double width = geoShape.getWidth();
    double length = geoShape.getLength();

    return std::max({(width / 2) * cos(theta) - (length / 2) * sin(theta) + latPosition,
                     (width / 2) * cos(theta) - (-length / 2) * sin(theta) + latPosition,
                     (-width / 2) * cos(theta) - (length / 2) * sin(theta) + latPosition,
                     (-width / 2) * cos(theta) - (-length / 2) * sin(theta) + latPosition});
}

double Obstacle::rearS(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    double lonPosition = getLonPosition(roadNetwork, timeStep);
    double width = geoShape.getWidth();
    double length = geoShape.getLength();
    double theta = getCurvilinearOrientation(roadNetwork, timeStep);

    // use minimum of all corners
    return std::min({(length / 2) * cos(theta) - (width / 2) * sin(theta) + lonPosition,
                     (length / 2) * cos(theta) - (-width / 2) * sin(theta) + lonPosition,
                     (-length / 2) * cos(theta) - (width / 2) * sin(theta) + lonPosition,
                     (-length / 2) * cos(theta) - (-width / 2) * sin(theta) + lonPosition});
}

double Obstacle::rightD(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    double latPos = getLatPosition(roadNetwork, timeStep);
    double width = geoShape.getWidth();
    double length = geoShape.getLength();
    double theta = getCurvilinearOrientation(roadNetwork, timeStep);

    return std::min({(width / 2) * cos(theta) - (length / 2) * sin(theta) + latPos,
                     (width / 2) * cos(theta) - (-length / 2) * sin(theta) + latPos,
                     (-width / 2) * cos(theta) - (length / 2) * sin(theta) + latPos,
                     (-width / 2) * cos(theta) - (-length / 2) * sin(theta) + latPos});
}

double Obstacle::leftD(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    double latPos = getLatPosition(roadNetwork, timeStep);
    double width = geoShape.getWidth();
    double length = geoShape.getLength();
    double theta = getCurvilinearOrientation(roadNetwork, timeStep);

    return std::max({(width / 2) * cos(theta) - (length / 2) * sin(theta) + latPos,
                     (width / 2) * cos(theta) - (-length / 2) * sin(theta) + latPos,
                     (-width / 2) * cos(theta) - (length / 2) * sin(theta) + latPos,
                     (-width / 2) * cos(theta) - (-length / 2) * sin(theta) + latPos});
}

double Obstacle::getLonPosition(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    if (getStateByTimeStep(timeStep)->getValidStates().lonPosition)
        return getStateByTimeStep(timeStep)->getLonPosition();
    convertPointToCurvilinear(roadNetwork, timeStep);
    return getStateByTimeStep(timeStep)->getLonPosition();
}

double Obstacle::getLatPosition(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    if (getStateByTimeStep(timeStep)->getValidStates().latPosition)
        return getStateByTimeStep(timeStep)->getLatPosition();
    convertPointToCurvilinear(roadNetwork, timeStep);
    return getStateByTimeStep(timeStep)->getLatPosition();
}

double Obstacle::getLonPosition(size_t timeStep, const std::shared_ptr<Lane> &refLane) {
    if (!(convertedPositions.count(timeStep) == 1 and
          convertedPositions[timeStep].count(refLane->getContainedLaneletIDs()) == 1)) {
        try {
            convertPointToCurvilinear(timeStep, refLane);
        } catch (...) {
            std::string refInfo;
            for (const auto &ref : refLane->getCurvilinearCoordinateSystem().referencePath())
                refInfo += "{" + std::to_string(ref.x()) + ", " + std::to_string(ref.y()) + "}, ";
            throw std::runtime_error(
                "Obstacle::getLonPosition Custom CCS - Curvilinear Projection Error - Obstacle ID: " +
                std::to_string(obstacleId) + " - Time Step: " + std::to_string(timeStep) +
                " - Reference Lane: " + refInfo);
        }
    }
    return convertedPositions[timeStep][refLane->getContainedLaneletIDs()][0];
}

double Obstacle::getLatPosition(size_t timeStep, const std::shared_ptr<Lane> &refLane) {
    if (!(convertedPositions.count(timeStep) == 1 and
          convertedPositions[timeStep].count(refLane->getContainedLaneletIDs()) == 1)) {
        try {
            convertPointToCurvilinear(timeStep, refLane);
        } catch (...) {
            std::string refInfo;
            for (const auto &ref : refLane->getCurvilinearCoordinateSystem().referencePath())
                refInfo += "{" + std::to_string(ref.x()) + ", " + std::to_string(ref.y()) + "}, ";
            throw std::runtime_error(
                "Obstacle::getLatPosition Custom CCS - Curvilinear Projection Error - Obstacle ID: " +
                std::to_string(obstacleId) + " - Time Step: " + std::to_string(timeStep) +
                " - Reference Lane: " + refInfo);
        }
    }
    return convertedPositions[timeStep][refLane->getContainedLaneletIDs()][1];
}

double Obstacle::getCurvilinearOrientation(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    if (getStateByTimeStep(timeStep)->getValidStates().curvilinearOrientation)
        return getStateByTimeStep(timeStep)->getCurvilinearOrientation();
    convertPointToCurvilinear(roadNetwork, timeStep);
    return getStateByTimeStep(timeStep)->getCurvilinearOrientation();
}

double Obstacle::getCurvilinearOrientation(size_t timeStep, const std::shared_ptr<Lane> &refLane) {
    if (!(convertedPositions.count(timeStep) == 1 and
          convertedPositions[timeStep].count(refLane->getContainedLaneletIDs()) == 1)) {
        try {
            convertPointToCurvilinear(timeStep, refLane);
        } catch (...) {
            std::string refInfo;
            for (const auto &ref : refLane->getCurvilinearCoordinateSystem().referencePath())
                refInfo += "{" + std::to_string(ref.x()) + ", " + std::to_string(ref.y()) + "}, ";
            throw std::runtime_error(
                "Obstacle::getCurvilinearOrientation Custom CCS - Curvilinear Projection Error - Obstacle ID: " +
                std::to_string(obstacleId) + " - Time Step: " + std::to_string(timeStep) +
                " - Reference Lane: " + refInfo);
        }
    }
    return convertedPositions[timeStep][refLane->getContainedLaneletIDs()][2];
}

size_t Obstacle::getFirstTrajectoryTimeStep() { return trajectoryPrediction.begin()->second->getTimeStep(); }

size_t Obstacle::getLastTrajectoryTimeStep() const {
    return trajectoryPrediction.begin()->second->getTimeStep() + getTrajectoryLength() - 1;
}

std::shared_ptr<Lane> Obstacle::getReferenceLane(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    return setReferenceLane(roadNetwork, timeStep);
}

std::shared_ptr<Lane> Obstacle::setReferenceLane(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    if (referenceLane.count(timeStep) == 1 and referenceLane.at(timeStep) != nullptr)
        return referenceLane.at(timeStep);
    else if (!existsOccupiedLanes(timeStep))
        setOccupiedLanes(roadNetwork, timeStep);

    std::vector<std::shared_ptr<Lane>> relevantOccupiedLanes;
    std::vector<std::shared_ptr<Lanelet>> relevantLanelets;
    // 1. check which currently occupied lanelets match direction
    if (getOccupiedLaneletsByShape(roadNetwork, timeStep).size() == 1)
        relevantLanelets.push_back(getOccupiedLaneletsByShape(roadNetwork, timeStep).front());
    else {
        for (const auto &lanelet : getOccupiedLaneletsByShape(roadNetwork, timeStep)) {
            auto curPointOrientation{lanelet->getOrientationAtPosition(getStateByTimeStep(timeStep)->getXPosition(),
                                                                       getStateByTimeStep(timeStep)->getYPosition())};
            if (abs(geometric_operations::subtractOrientations(curPointOrientation,
                                                               getStateByTimeStep(timeStep)->getGlobalOrientation())) <
                laneOrientationThresholdInitial)
                relevantLanelets.push_back(lanelet);
        }
    }
    // 2. neglect lanes containing lanelets which do not match
    for (const auto &lane : getOccupiedLanes(roadNetwork, timeStep))
        for (const auto &lanelet : relevantLanelets)
            if (lane->getContainedLaneletIDs().count(lanelet->getId()) == 1)
                relevantOccupiedLanes.push_back(lane);
    // 3. calc num occupancies starting from provided time step
    if (relevantOccupiedLanes.size() == 1) { // only one relevant lane is occupied
        referenceLane[timeStep] = relevantOccupiedLanes.at(0);
    } else if (relevantOccupiedLanes.size() > 1) { // iterate over all time steps and check whether orientation fits and
        // choose lane with most occupancies; if initial lane is adjacent use this lane
        std::map<size_t, size_t> numOccupancies;
        for (size_t newTimeStep{timeStep}; newTimeStep <= getLastTrajectoryTimeStep(); ++newTimeStep) {
            std::multimap<double, size_t> bestOccupancies;
            for (const auto &lane : relevantOccupiedLanes) {
                auto curPointOrientation{lane->getOrientationAtPosition(
                    getStateByTimeStep(newTimeStep)->getXPosition(), getStateByTimeStep(newTimeStep)->getYPosition())};
                auto orientationDif{abs(geometric_operations::subtractOrientations(
                    curPointOrientation, getStateByTimeStep(newTimeStep)->getGlobalOrientation()))};
                if (orientationDif < laneOrientationThreshold)
                    bestOccupancies.insert({orientationDif, lane->getId()});
            }
            for (const auto &elem : bestOccupancies) {
                if (elem.first != bestOccupancies.begin()->first)
                    break;
                numOccupancies[elem.second]++;
            }
        }
        if (!numOccupancies.empty()) { // find lane with most occupancies (orientation must also match)
            std::vector<size_t> ids;
            std::multimap<int, size_t> multimap;
            std::vector<std::shared_ptr<Lane>> referenceLaneCandidates;
            for (auto &&occupancy : numOccupancies)
                multimap.insert(std::make_pair(occupancy.second, occupancy.first));
            auto it1 = multimap.rbegin(); // get the elem with the highest key
            auto range = multimap.equal_range(it1->first);
            for (auto it2 = range.first; it2 != range.second; ++it2)
                ids.push_back(it2->second);
            for (const auto &relLane : relevantOccupiedLanes)
                for (const auto &canLaneId : ids)
                    if (relLane->getId() == canLaneId)
                        referenceLaneCandidates.push_back(relLane);
            if (referenceLaneCandidates.size() > 1) {
                std::vector<double> avgLaneOrientationChange;
                avgLaneOrientationChange.reserve(referenceLaneCandidates.size());
                for (size_t i{0}; i < referenceLaneCandidates.size(); ++i) {
                    std::vector<double> orientation = referenceLaneCandidates[i]->getOrientation();
                    double tmp{0};
                    for (size_t j{1}; j < orientation.size(); ++j)
                        tmp += abs(geometric_operations::subtractOrientations(orientation[j], orientation[j - 1]));
                    avgLaneOrientationChange.push_back(
                        tmp / static_cast<double>(referenceLaneCandidates[i]->getCenterVertices().size()));
                }
                auto laneIdx{
                    std::distance(avgLaneOrientationChange.begin(),
                                  std::min_element(avgLaneOrientationChange.begin(), avgLaneOrientationChange.end()))};
                referenceLane[timeStep] = referenceLaneCandidates.at(static_cast<unsigned long>(laneIdx));

            } else if (referenceLaneCandidates.size() == 1)
                referenceLane[timeStep] = referenceLaneCandidates.front();
        }
    }
    // if no reference lane found: check previous reference lane, if no previous exist try to use future reference lane
    if (referenceLane.count(timeStep) != 1) {
        if (referenceLane.count(timeStep - 1) == 1 and referenceLane.at(timeStep - 1) != nullptr)
            referenceLane[timeStep] = referenceLane.at(timeStep - 1);
        else
            for (size_t newTimeStep{timeStep + 1}; newTimeStep <= getLastTrajectoryTimeStep(); ++newTimeStep) {
                referenceLane[timeStep] = getReferenceLane(roadNetwork, newTimeStep);
            }
    }
    if (referenceLane.count(timeStep) == 0 or referenceLane.at(timeStep) == nullptr)
        throw std::runtime_error("Obstacle::setReferenceLane: No matching referenceLane found! Obstacle ID " +
                                 std::to_string(getId()) + " at time step " + std::to_string(timeStep));
    return referenceLane.at(timeStep);
}

void Obstacle::convertPointToCurvilinear(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    auto curReferenceLane{getReferenceLane(roadNetwork, timeStep)};
    try {
        convertPointToCurvilinear(timeStep, curReferenceLane);
        getStateByTimeStep(timeStep)->setLonPosition(
            convertedPositions[timeStep][curReferenceLane->getContainedLaneletIDs()][0]);
        getStateByTimeStep(timeStep)->setLatPosition(
            convertedPositions[timeStep][curReferenceLane->getContainedLaneletIDs()][1]);
        getStateByTimeStep(timeStep)->setCurvilinearOrientation(
            convertedPositions[timeStep][curReferenceLane->getContainedLaneletIDs()][2]);
    } catch (...) {
        std::string refInfo;
        for (const auto &ref : curReferenceLane->getCurvilinearCoordinateSystem().referencePath())
            refInfo += "{" + std::to_string(ref.x()) + ", " + std::to_string(ref.y()) + "}, ";
        throw std::runtime_error("Obstacle::convertPointToCurvilinear: Curvilinear Projection Error - Obstacle ID: " +
                                 std::to_string(obstacleId) + " - Time Step: " + std::to_string(timeStep) +
                                 " - Reference Lane: " + refInfo +
                                 " - x-position: " + std::to_string(getStateByTimeStep(timeStep)->getXPosition()) +
                                 " - y-position: " + std::to_string(getStateByTimeStep(timeStep)->getYPosition()));
    }
}

void Obstacle::interpolateAcceleration(size_t timeStep, double timeStepSize) {
    if (getStateByTimeStep(timeStep)->getValidStates().acceleration)
        return;
    if (!timeStepExists(timeStep - 1)) {
        getStateByTimeStep(timeStep)->setAcceleration(0);
        return;
    }
    double curVelocity{getStateByTimeStep(timeStep)->getVelocity()};
    double prevVelocity{getStateByTimeStep(timeStep - 1)->getVelocity()};
    getStateByTimeStep(timeStep)->setAcceleration((curVelocity - prevVelocity) / timeStepSize);
}

const std::vector<vertex> &Obstacle::getRoute() const { return route; }

void Obstacle::setRoute(const std::vector<vertex> &newRoute) { route = newRoute; }

void Obstacle::setOccupiedLanes(const std::vector<std::shared_ptr<Lane>> &lanes, size_t timeStep) {
    if (occupiedLanes.count(timeStep) == 0)
        occupiedLanes[timeStep] = lanes;
}

void Obstacle::setOccupiedLanes(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    auto lanelets{getOccupiedLaneletsByShape(roadNetwork, timeStep)};
    std::vector<std::shared_ptr<Lane>> occLanes{
        lanelet_operations::createLanesBySingleLanelets(lanelets, roadNetwork, fieldOfViewRear, fieldOfViewFront)};
    occupiedLanes[timeStep] = occLanes;
}

std::vector<std::shared_ptr<Lane>> Obstacle::getDrivingPathLanes(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                                 size_t timeStep) {
    auto occLanes{getOccupiedLanes(roadNetwork, timeStep)};
    if (occLanes.size() == 1)
        return occLanes;
    else {
        std::vector<std::shared_ptr<Lane>> relevantLanes;
        auto occLanelets{getOccupiedLaneletsByShape(roadNetwork, timeStep)};
        for (const auto &lanelet : occLanes) {
            if (lanelet->getId() == getReferenceLane(roadNetwork, timeStep)->getId()) {
                relevantLanes.push_back(lanelet);
                continue;
            }
            if (lanelet_operations::areLaneletsInDirectlyAdjacentLanes(getReferenceLane(roadNetwork, timeStep), lanelet,
                                                                       occLanelets))
                relevantLanes.push_back(lanelet);
        }
        return {relevantLanes};
    }
}

std::vector<std::shared_ptr<Lane>> Obstacle::getOccupiedLanes(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                              size_t timeStep) {
    if (occupiedLanes[timeStep].empty())
        setOccupiedLanes(roadNetwork, timeStep);
    return occupiedLanes[timeStep];
}

void Obstacle::computeLanes(const std::shared_ptr<RoadNetwork> &roadNetwork, bool considerHistory) {
    const size_t timeStamp{currentState->getTimeStep()};
    auto lanelets{getOccupiedLaneletsByShape(roadNetwork, timeStamp)};
    auto lanes{
        lanelet_operations::createLanesBySingleLanelets(lanelets, roadNetwork, fieldOfViewRear, fieldOfViewFront)};
    setOccupiedLanes(lanes, timeStamp);
    if (!isStatic) {
        for (const auto &time : getPredictionTimeSteps())
            setOccupiedLanes(roadNetwork, time);
        if (considerHistory)
            for (const auto &time : getHistoryTimeSteps())
                setOccupiedLanes(roadNetwork, time);
    }
}

void Obstacle::setCurvilinearStates(const std::shared_ptr<RoadNetwork> &roadNetwork) {
    if (!currentState->getValidStates().lonPosition)
        convertPointToCurvilinear(roadNetwork, currentState->getTimeStep());
    if (!isStatic)
        for (const auto &timeStep : getPredictionTimeSteps())
            if (!getStateByTimeStep(timeStep)->getValidStates().lonPosition)
                convertPointToCurvilinear(roadNetwork, timeStep);
}
bool Obstacle::existsOccupiedLanes(size_t timeStep) { return occupiedLanes.count(timeStep) >= 1; }

const polygon_type &Obstacle::getFov() const { return fov; }

void Obstacle::setFov(const polygon_type &fov) { Obstacle::fov = fov; }
