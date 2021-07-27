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

std::vector<size_t> Obstacle::getPredictionTimeSteps() {
    std::vector<size_t> timeSteps;
    boost::copy(trajectoryPrediction | boost::adaptors::map_keys, std::back_inserter(timeSteps));
    return timeSteps;
}

std::vector<size_t> Obstacle::getTimeSteps() {
    std::vector<size_t> timeSteps{getPredictionTimeSteps()};
    if (timeSteps.front() != currentState->getTimeStep())
        timeSteps.insert(timeSteps.begin(), currentState->getTimeStep());
    return timeSteps;
}

void Obstacle::setReferenceLane(const std::shared_ptr<RoadNetwork> &roadNetwork) {
    referenceLane = nullptr;
    std::vector<std::shared_ptr<Lane>> referenceLaneCandidates;
    // find relevant lanes -> occupied at first time step + occupied last time step and first time step adjacent
    std::vector<std::shared_ptr<Lane>> relevantOccupiedLanes;
    auto occupiedLaneletsFirstTimeStep{getOccupiedLanelets(roadNetwork, currentState->getTimeStep())};
    std::vector<std::shared_ptr<Lanelet>> occupiedLaneletsFirstTimeStepAdjacent{};

    for (const auto &la : occupiedLaneletsFirstTimeStep) {
        auto curPointOrientation{
            la->getOrientationAtPosition(currentState->getXPosition(), currentState->getYPosition())};
        if (abs(geometric_operations::subtractOrientations(
                curPointOrientation, currentState->getGlobalOrientation())) >= laneOrientationThresholdInitial)
            continue;
        auto adjacent{lanelet_operations::adjacentLanelets(la)};
        for (const auto &adj : adjacent)
            if (std::find(occupiedLaneletsFirstTimeStepAdjacent.begin(), occupiedLaneletsFirstTimeStepAdjacent.end(),
                          adj) == occupiedLaneletsFirstTimeStepAdjacent.end())
                occupiedLaneletsFirstTimeStepAdjacent.push_back(adj);
    }

    std::set<size_t> consideredLanes;
    for (const auto &laneLast : occupiedLanes[getLastTrajectoryTimeStep()])
        for (const auto &laneletFirst : occupiedLaneletsFirstTimeStepAdjacent) {
            if (laneLast->getContainedLaneletIDs().count(laneletFirst->getId()) != 0u and
                consideredLanes.find(laneLast->getId()) == consideredLanes.end()) {
                relevantOccupiedLanes.push_back(laneLast);
                consideredLanes.insert(laneLast->getId());
            }
        }
    if (relevantOccupiedLanes.size() == 1) { // only one lane is occupied over complete trajectory -> this is reference
        referenceLane = relevantOccupiedLanes.at(0);
    } else if (relevantOccupiedLanes.size() > 1) { // iterate over all time steps and check whether orientation fits and
                                                   // choose lane with most occupancies; if
                                                   // initial lane is adjacent use this lane
        std::map<size_t, size_t> numOccupancies;
        for (const auto &timeStep : getTimeSteps()) {
            for (const auto &la : occupiedLanes[timeStep]) {
                if (!std::any_of(relevantOccupiedLanes.begin(), relevantOccupiedLanes.end(),
                                 [la](const std::shared_ptr<Lane> &rel) { return rel->getId() == la->getId(); }))
                    continue;
                auto curPointOrientation{la->getOrientationAtPosition(getStateByTimeStep(timeStep)->getXPosition(),
                                                                      getStateByTimeStep(timeStep)->getYPosition())};
                if (abs(geometric_operations::subtractOrientations(
                        curPointOrientation, getStateByTimeStep(timeStep)->getGlobalOrientation())) <
                    laneOrientationThreshold)
                    numOccupancies[la->getId()]++;
            }
        }
        if (!numOccupancies.empty()) {
            std::vector<size_t> ids;
            std::multimap<int, size_t> m2;
            for (auto &&i : numOccupancies)
                m2.insert(std::make_pair(i.second, i.first));
            auto it1 = m2.rbegin(); // get the elem with the highest key
            auto range = m2.equal_range(it1->first);
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
                referenceLane = referenceLaneCandidates.at(static_cast<unsigned long>(laneIdx));

            } else if (referenceLaneCandidates.size() == 1)
                referenceLane = referenceLaneCandidates.front();
        } else {
            // if at all time steps only a single lanelet is occupied -> use arbitrary lane
            size_t laneletIDFirstTimeStep;
            bool useArbitraryLane{true};
            if (occupiedLaneletsFirstTimeStep.size() == 1)
                laneletIDFirstTimeStep = occupiedLaneletsFirstTimeStep.at(0)->getId();
            for (const auto &time : getPredictionTimeSteps())
                if (!(getOccupiedLanelets(roadNetwork, time).size() == 1 and
                      getOccupiedLanelets(roadNetwork, time).at(0)->getId() == laneletIDFirstTimeStep)) {
                    useArbitraryLane = false;
                    break;
                }
            if (useArbitraryLane)
                referenceLane = relevantOccupiedLanes.at(0);
        }
        if (referenceLane == nullptr)
            throw std::runtime_error("Obstacle::setReferenceLane: No matching referenceLane found! Obstacle ID " +
                                     std::to_string(getId()));
    } else
        throw std::runtime_error("Obstacle::setReferenceLane: No referenceLane found! Obstacle ID " +
                                 std::to_string(getId()));
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

double Obstacle::frontS(size_t timeStep, const std::shared_ptr<Lane> &refLane) {
    try {
        Eigen::Vector2d convertedPoint = refLane->getCurvilinearCoordinateSystem()->convertToCurvilinearCoords(
            getStateByTimeStep(timeStep)->getXPosition(), getStateByTimeStep(timeStep)->getYPosition());
        double theta = getStateByTimeStep(timeStep)->getGlobalOrientation() -
                       refLane->getOrientationAtPosition(getStateByTimeStep(timeStep)->getXPosition(),
                                                         getStateByTimeStep(timeStep)->getYPosition());
        double s = convertedPoint.x();
        double width = geoShape.getWidth();
        double length = geoShape.getLength();

        // use maximum of all corners
        return std::max({(length / 2) * cos(theta) - (width / 2) * sin(theta) + s,
                         (length / 2) * cos(theta) - (-width / 2) * sin(theta) + s,
                         (-length / 2) * cos(theta) - (width / 2) * sin(theta) + s,
                         (-length / 2) * cos(theta) - (-width / 2) * sin(theta) + s});
    } catch (...) {
        throw std::runtime_error(
            "Obstacle::frontS Custom CCS - Curvilinear Projection Error - Obstacle ID: " + std::to_string(id) +
            " - Time Step: " + std::to_string(timeStep) + " - Reference Lane: " + std::to_string(refLane->getId()));
    }
}

double Obstacle::rearS(size_t timeStep, const std::shared_ptr<Lane> &refLane) {
    try {
        Eigen::Vector2d convertedPoint = refLane->getCurvilinearCoordinateSystem()->convertToCurvilinearCoords(
            getStateByTimeStep(timeStep)->getXPosition(), getStateByTimeStep(timeStep)->getYPosition());
        double theta = getStateByTimeStep(timeStep)->getGlobalOrientation() -
                       refLane->getOrientationAtPosition(getStateByTimeStep(timeStep)->getXPosition(),
                                                         getStateByTimeStep(timeStep)->getYPosition());
        double s = convertedPoint.x();
        double width = geoShape.getWidth();
        double length = geoShape.getLength();
        // use minimum of all corners
        return std::min({(length / 2) * cos(theta) - (width / 2) * sin(theta) + s,
                         (length / 2) * cos(theta) - (-width / 2) * sin(theta) + s,
                         (-length / 2) * cos(theta) - (width / 2) * sin(theta) + s,
                         (-length / 2) * cos(theta) - (-width / 2) * sin(theta) + s});
    } catch (...) {
        throw std::runtime_error(
            "Obstacle::rearS Custom CCS - Curvilinear Projection Error - Obstacle ID: " + std::to_string(id) +
            " - Time Step: " + std::to_string(timeStep) + " - Reference Lane: " + std::to_string(refLane->getId()));
    }
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

double Obstacle::getLonPosition(size_t timeStep, const std::shared_ptr<Lane> &refLane) const {
    try {
        Eigen::Vector2d convertedPoint = refLane->getCurvilinearCoordinateSystem()->convertToCurvilinearCoords(
            getStateByTimeStep(timeStep)->getXPosition(), getStateByTimeStep(timeStep)->getYPosition());
        return convertedPoint.x();
    } catch (...) {
        throw std::runtime_error(
            "Obstacle::getLonPosition Custom CCS - Curvilinear Projection Error - Obstacle ID: " + std::to_string(id) +
            " - Time Step: " + std::to_string(timeStep) + " - Reference Lane: " + std::to_string(refLane->getId()));
    }
}

double Obstacle::getLatPosition(size_t timeStep, const std::shared_ptr<Lane> &refLane) const {
    try {
        Eigen::Vector2d convertedPoint = refLane->getCurvilinearCoordinateSystem()->convertToCurvilinearCoords(
            getStateByTimeStep(timeStep)->getXPosition(), getStateByTimeStep(timeStep)->getYPosition());
        return convertedPoint.y();
    } catch (...) {
        throw std::runtime_error(
            "Obstacle::getLatPosition Custom CCS - Curvilinear Projection Error - Obstacle ID: " + std::to_string(id) +
            " - Time Step: " + std::to_string(timeStep) + " - Reference Lane: " + std::to_string(refLane->getId()));
    }
}

double Obstacle::getCurvilinearOrientation(size_t timeStep) const {
    if (getStateByTimeStep(timeStep)->getValidStates().curvilinearOrientation)
        return getStateByTimeStep(timeStep)->getCurvilinearOrientation();
    convertPointToCurvilinear(timeStep);
    return getStateByTimeStep(timeStep)->getCurvilinearOrientation();
}

size_t Obstacle::getFirstTrajectoryTimeStep() { return trajectoryPrediction.begin()->second->getTimeStep(); }

size_t Obstacle::getLastTrajectoryTimeStep() {
    return trajectoryPrediction.begin()->second->getTimeStep() + getTrajectoryLength() - 1;
}

std::shared_ptr<Lane> Obstacle::getReferenceLane() const { return referenceLane; }

void Obstacle::convertPointToCurvilinear(size_t timeStep) const {
    try {
        Eigen::Vector2d convertedPoint = referenceLane->getCurvilinearCoordinateSystem()->convertToCurvilinearCoords(
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
    if (occupiedLanes.count(timeStep) == 0)
        occupiedLanes[timeStep] = lanes;
}

void Obstacle::setOccupiedLanes(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep,
                                std::shared_ptr<size_t> idCounter, double fovFront) {
    auto lanelets{getOccupiedLanelets(roadNetwork, timeStep)};
    std::vector<std::shared_ptr<Lane>> occLanes{
        lanelet_operations::createLanesBySingleLanelets(lanelets, idCounter, roadNetwork, fieldOfViewRear, fovFront)};
    occupiedLanes[timeStep] = occLanes;
}
double Obstacle::approximateFieldOfView() {
    double fovFront{fieldOfViewFront};
    if (fieldOfViewFront <
        static_cast<double>(trajectoryPrediction.size()) * dt * currentState->getVelocity() * fovApproximationFactor) {
        double maxV{0.0};
        for (const auto &state : trajectoryPrediction)
            if (state.second->getVelocity() > maxV)
                maxV = state.second->getVelocity();
        fovFront = maxV * static_cast<double>(trajectoryPrediction.size()) * dt;
    }
    return fovFront;
}

std::vector<std::shared_ptr<Lane>> Obstacle::getDrivingPathLanes(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                                 size_t timeStep, std::shared_ptr<size_t> idCounter) {
    auto occLanes{getOccupiedLanes(roadNetwork, timeStep, idCounter)};
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

std::vector<std::shared_ptr<Lane>> Obstacle::getOccupiedLanes(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                              size_t timeStep, std::shared_ptr<size_t> idCounter) {
    if (occupiedLanes[timeStep].empty()) {
        double fovFront = approximateFieldOfView();
        setOccupiedLanes(roadNetwork, timeStep, idCounter, fovFront);
    }
    return occupiedLanes[timeStep];
}

void Obstacle::computeLanes(const std::shared_ptr<RoadNetwork> &roadNetwork, std::shared_ptr<size_t> idCounter) {
    const size_t timeStamp{currentState->getTimeStep()};
    auto lanelets{getOccupiedLanelets(roadNetwork, timeStamp)};
    double fovFront = approximateFieldOfView();
    auto lanes{
        lanelet_operations::createLanesBySingleLanelets(lanelets, idCounter, roadNetwork, fieldOfViewRear, fovFront)};
    setOccupiedLanes(lanes, timeStamp);
    for (const auto &la : lanelets) // add initial adjacent lanelets to road network; ignore return value
        lanelet_operations::createLanesBySingleLanelets(lanelet_operations::adjacentLanelets(la), idCounter,
                                                        roadNetwork, fieldOfViewRear, fovFront);
    if (!isStatic) {
        for (const auto &time : getPredictionTimeSteps())
            setOccupiedLanes(roadNetwork, time, idCounter, fovFront);
        setReferenceLane(roadNetwork);
    }
}