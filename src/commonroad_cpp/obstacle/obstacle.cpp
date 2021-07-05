//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "obstacle.h"
#include "../geometry/geometric_operations.h"
#include "../roadNetwork/lanelet/lanelet_operations.h"

#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <cmath>
#include <utility>

Obstacle::Obstacle(size_t id, bool isStatic, std::shared_ptr<State> currentState, ObstacleType obstacleType,
                   double vMax, double aMax, double aMaxLong, double aMinLong, double reactionTime,
                   std::map<size_t, std::shared_ptr<State>> trajectoryPrediction, double length, double width,
                   std::vector<vertex> route)
    : id(id), isStatic(isStatic), currentState(std::move(currentState)), obstacleType(obstacleType), vMax(vMax),
      aMax(aMax), aMaxLong(aMaxLong), aMinLong(aMinLong), reactionTime(reactionTime),
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

void Obstacle::setId(const size_t obstacleId) { id = obstacleId; }

void Obstacle::setIsStatic(bool st) {
    isStatic = st;
    if (st) {
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

size_t Obstacle::getId() const { return id; }

bool Obstacle::getIsStatic() const { return isStatic; }

const std::shared_ptr<State> &Obstacle::getCurrentState() const { return currentState; }

bool Obstacle::timeStepExists(size_t timeStep) {
    return (trajectoryPrediction.count(timeStep) == 1 or currentState->getTimeStep() == timeStep or
            history.count(timeStep) == 1);
}

std::shared_ptr<State> Obstacle::getStateByTimeStep(size_t timeStep) const {
    if (trajectoryPrediction.count(timeStep) == 1)
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

std::vector<size_t> Obstacle::getPredictionTimeSteps(){
    std::vector<size_t> timeSteps;
    boost::copy(trajectoryPrediction | boost::adaptors::map_keys, std::back_inserter(timeSteps));
    return timeSteps;
}

void Obstacle::setReferenceLane(std::shared_ptr<RoadNetwork> roadNetwork) {
    referenceLane = nullptr;
    // only one lane is occupied over complete trajectory -> this is reference
    auto allOccupiedLanes{getOccupiedLanes()};
    if (allOccupiedLanes.size() == 1) {
        referenceLane = allOccupiedLanes.at(0);
        return;
    } else { // iterate over all time steps and check whether orientation fits and choose lane with most occupancies; if
             // initial lane is adjacent use this lane
        std::map<size_t, size_t> numOccupancies;
        for (const auto &timeStep : getPredictionTimeSteps()) {
            for (const auto &la : occupiedLanes[timeStep]) {
                auto curPointOrientation{la->getOrientationAtPosition(
                    getStateByTimeStep(timeStep)->getXPosition(), getStateByTimeStep(timeStep)->getYPosition())};
                if (abs(curPointOrientation - getStateByTimeStep(timeStep)->getGlobalOrientation()) < 0.35) //update this since orientation representation is two-folded
                    numOccupancies[la->getId()]++;
            }
        }
        auto pr = std::max_element(
            std::begin(numOccupancies), std::end(numOccupancies),
            [](std::pair<size_t, size_t> p1, const std::pair<size_t, size_t> &p2) { return p1.second < p2.second; });
        for (const auto &laneAtTimeStep : occupiedLanes) {
            for (const auto &la : laneAtTimeStep.second)
                if (la->getId() == pr->first) {
                    referenceLane = la;
                    break;
                }
            if (referenceLane != nullptr)
                break;
        }
        // if lane is not occupied at first and last time step -> error
        auto occupiedLaneletsFirstTimeStep{getOccupiedLanelets(roadNetwork, getFirstTrajectoryTimeStep())};
        auto occupiedLaneletsLastTimeStep{getOccupiedLanelets(roadNetwork, getLastTrajectoryTimeStep())};
        if (referenceLane == nullptr)
            throw std::runtime_error("Obstacle::setReferenceLane: No referenceLane found! Obstacle ID " +
                                     std::to_string(getId()));
        else {
            auto refIDs{referenceLane->getContainedLaneletIDs()};
            if (!std::any_of(
                    occupiedLaneletsFirstTimeStep.begin(), occupiedLaneletsFirstTimeStep.end(),
                    [refIDs](std::shared_ptr<Lanelet> la) { return refIDs.find(la->getId()) != refIDs.end(); }) or
                !std::any_of(
                    occupiedLaneletsLastTimeStep.begin(), occupiedLaneletsLastTimeStep.end(),
                    [refIDs](std::shared_ptr<Lanelet> la) { return refIDs.find(la->getId()) != refIDs.end(); }))
                throw std::runtime_error("Obstacle::setReferenceLane: No valid referenceLane found! Obstacle ID " +
                                         std::to_string(getId()));
        }
    }
}

std::map<size_t, std::shared_ptr<State>> Obstacle::getTrajectoryPrediction() const { return trajectoryPrediction; }

size_t Obstacle::getTrajectoryLength() { return trajectoryPrediction.size(); }

polygon_type Obstacle::getOccupancyPolygonShape(size_t timeStep) {
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
    return polygonShape;
}

Shape &Obstacle::getGeoShape() { return geoShape; }

std::vector<std::shared_ptr<Lanelet>> Obstacle::getOccupiedLanelets(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                                    size_t timeStep) {
    if (occupiedLanelets.find(timeStep) != occupiedLanelets.end())
        return occupiedLanelets.at(timeStep);
    polygon_type polygonShape{getOccupancyPolygonShape(timeStep)};
    std::vector<std::shared_ptr<Lanelet>> occupied{roadNetwork->findOccupiedLaneletsByShape(polygonShape)};
    occupiedLanelets.insert(std::pair<int, std::vector<std::shared_ptr<Lanelet>>>(timeStep, occupied));

    return occupied;
}

double Obstacle::frontS(size_t timeStep) {
    double s = getLonPosition(timeStep);
    double width = geoShape.getWidth();
    double length = geoShape.getLength();
    double theta = getStateByTimeStep(timeStep)->getCurvilinearOrientation();

    // use maximum of all corners
    return std::max({(length / 2) * cos(theta) - (width / 2) * sin(theta) + s,
                     (length / 2) * cos(theta) - (-width / 2) * sin(theta) + s,
                     (-length / 2) * cos(theta) - (width / 2) * sin(theta) + s,
                     (-length / 2) * cos(theta) - (-width / 2) * sin(theta) + s});
}

double Obstacle::rearS(size_t timeStep) {
    double s = getLonPosition(timeStep);
    double width = geoShape.getWidth();
    double length = geoShape.getLength();
    double theta = getStateByTimeStep(timeStep)->getCurvilinearOrientation();

    // use minimum of all corners
    return std::min({(length / 2) * cos(theta) - (width / 2) * sin(theta) + s,
                     (length / 2) * cos(theta) - (-width / 2) * sin(theta) + s,
                     (-length / 2) * cos(theta) - (width / 2) * sin(theta) + s,
                     (-length / 2) * cos(theta) - (-width / 2) * sin(theta) + s});
}

double Obstacle::getLonPosition(size_t timeStep) const {
    if (getStateByTimeStep(timeStep)->getValidStates().lonPosition)
        return getStateByTimeStep(timeStep)->getLonPosition();
    convertPointToCurvilinear(timeStep);
    return getStateByTimeStep(timeStep)->getLonPosition();
}

double Obstacle::getLatPosition(size_t timeStep) const {
    if (getStateByTimeStep(timeStep)->getValidStates().latPosition)
        return getStateByTimeStep(timeStep)->getLatPosition();
    convertPointToCurvilinear(timeStep);
    return getStateByTimeStep(timeStep)->getLatPosition();
}

double Obstacle::getCurvilinearOrientation(size_t timeStep) const {
    if (getStateByTimeStep(timeStep)->getValidStates().curvilinearOrientation)
        return getStateByTimeStep(timeStep)->getCurvilinearOrientation();
    convertPointToCurvilinear(timeStep);
    return getStateByTimeStep(timeStep)->getCurvilinearOrientation();
}

std::vector<std::shared_ptr<Lane>> Obstacle::getOccupiedLanes(size_t timeStep) { return occupiedLanes.at(timeStep); }

size_t Obstacle::getFirstTrajectoryTimeStep() { return trajectoryPrediction.begin()->second->getTimeStep(); }

size_t Obstacle::getLastTrajectoryTimeStep() {
    return trajectoryPrediction.begin()->second->getTimeStep() + getTrajectoryLength() - 1;
}

std::shared_ptr<Lane> Obstacle::getReferenceLane() const { return referenceLane; }

void Obstacle::convertPointToCurvilinear(size_t timeStep) const {
    try {
        Eigen::Vector2d convertedPoint = referenceLane->getCurvilinearCoordinateSystem().convertToCurvilinearCoords(
            getStateByTimeStep(timeStep)->getXPosition(), getStateByTimeStep(timeStep)->getYPosition());
        getStateByTimeStep(timeStep)->setLonPosition(convertedPoint.x());
        getStateByTimeStep(timeStep)->setLatPosition(convertedPoint.y());
        double theta = getStateByTimeStep(timeStep)->getGlobalOrientation() -
                       getReferenceLane()->getOrientationAtPosition(getStateByTimeStep(timeStep)->getXPosition(),
                                                                    getStateByTimeStep(timeStep)->getYPosition());
        getStateByTimeStep(timeStep)->setCurvilinearOrientation(theta);
    } catch (...) {
        throw std::runtime_error("Curvilinear Projection Error - Obstacle ID: " + std::to_string(id) +
                                 " - Time Step: " + std::to_string(timeStep) +
                                 " - Reference Lane: " + std::to_string(referenceLane->getId()));
    }
}

void Obstacle::interpolateAcceleration(size_t timeStep) {
    if (getStateByTimeStep(timeStep)->getValidStates().acceleration)
        return;
    if (!timeStepExists(timeStep - 1))
        getStateByTimeStep(timeStep)->setAcceleration(0);
    double curVelocity{getStateByTimeStep(timeStep)->getVelocity()};
    double prevVelocity{getStateByTimeStep(timeStep - 1)->getVelocity()};
    getStateByTimeStep(timeStep)->setAcceleration((curVelocity - prevVelocity) / dt);
}

const std::vector<vertex> &Obstacle::getRoute() const { return route; }

void Obstacle::setRoute(const std::vector<vertex> &newRoute) { route = newRoute; }

void Obstacle::setOccupiedLanes(const std::vector<std::shared_ptr<Lane>> &lanes, size_t timeStep) {
    occupiedLanes[timeStep] = lanes;
}

void Obstacle::setOccupiedLanes(const std::shared_ptr<RoadNetwork> roadNetwork, size_t timeStep) {
    if (occupiedLanes.count(timeStep))
        return; // time step was already computed
    std::vector<std::shared_ptr<Lane>> occupied;
    std::vector<std::shared_ptr<Lanelet>> lanelets{getOccupiedLanelets(roadNetwork, timeStep)};
    for (const auto &lane : roadNetwork->getLanes()) {
        bool laneIsOccupied{false};
        for (const auto &laneletLane : lane->getContainedLanelets()) {
            for (const auto &laneletOccupied : lanelets) {
                if (laneletLane->getId() == laneletOccupied->getId()) {
                    occupied.push_back(lane);
                    laneIsOccupied = true;
                    break;
                }
            }
            if (laneIsOccupied)
                break;
        }
    }
    occupiedLanes[timeStep] = occupied;
}

std::vector<std::shared_ptr<Lane>> Obstacle::getDrivingPathLanes(std::shared_ptr<RoadNetwork> roadNetwork,
                                                                 size_t timeStep) {
    auto occLanes{occupiedLanes[timeStep]};
    if (occLanes.size() == 1)
        return occLanes;
    else {
        std::vector<std::shared_ptr<Lane>> relevantLanes;
        auto occLanelets{getOccupiedLanelets(roadNetwork, timeStep)};
        for (const auto &la : occLanes) {
            if (la->getId() == referenceLane->getId()) {
                relevantLanes.push_back(la);
                continue;
            }
            if (lanelet_operations::adjacentLanes(referenceLane, la, occLanelets))
                relevantLanes.push_back(la);
        }
        return {relevantLanes};
    }
}

std::vector<std::shared_ptr<Lane>> Obstacle::getOccupiedLanes() {
    std::set<size_t> ids;
    std::vector<std::shared_ptr<Lane>> lanes;
    for (const auto &laneAtTimeStep : occupiedLanes)
        for (const auto &la : laneAtTimeStep.second)
            if (ids.find(la->getId()) == ids.end()) {
                lanes.push_back(la);
                ids.insert(la->getId());
            }

    return lanes;
}