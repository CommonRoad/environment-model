//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/geometry/curvilinear_coordinate_system.h>

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
#include <commonroad_cpp/geometry/geometric_operations.h>
#include <commonroad_cpp/geometry/rectangle.h>
#include <commonroad_cpp/geometry/shape.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/obstacle_operations.h>
#include <commonroad_cpp/obstacle/state.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <commonroad_cpp/roadNetwork/road_network.h>

static std::array<double, 4> rotatedCornerLatitudes(const Rectangle &rect, const double theta) {
    const double width = rect.getWidth();
    const double length = rect.getLength();

    return {(width / 2) * cos(theta) - (length / 2) * sin(theta), (width / 2) * cos(theta) - (-length / 2) * sin(theta),
            (-width / 2) * cos(theta) - (length / 2) * sin(theta),
            (-width / 2) * cos(theta) - (-length / 2) * sin(theta)};
}

static std::array<double, 4> rotatedCornerLongitudes(const Rectangle &rect, const double theta) {
    const double width = rect.getWidth();
    const double length = rect.getLength();

    return {(length / 2) * cos(theta) - (width / 2) * sin(theta), (length / 2) * cos(theta) - (-width / 2) * sin(theta),
            (-length / 2) * cos(theta) - (width / 2) * sin(theta),
            (-length / 2) * cos(theta) - (-width / 2) * sin(theta)};
}

static double rotatedMinimumLatitude(const Rectangle &rect, const double theta) {
    const auto latitudes = rotatedCornerLatitudes(rect, theta);
    return *std::min_element(latitudes.cbegin(), latitudes.cend());
}

static double rotatedMaximumLatitude(const Rectangle &rect, const double theta) {
    const auto latitudes = rotatedCornerLatitudes(rect, theta);
    return *std::max_element(latitudes.cbegin(), latitudes.cend());
}

static double rotatedMinimumLongitude(const Rectangle &rect, const double theta) {
    const auto longitudes = rotatedCornerLongitudes(rect, theta);
    return *std::min_element(longitudes.cbegin(), longitudes.cend());
}

static double rotatedMaximumLongitude(const Rectangle &rect, const double theta) {
    const auto longitudes = rotatedCornerLongitudes(rect, theta);
    return *std::max_element(longitudes.cbegin(), longitudes.cend());
}

Obstacle::Obstacle(size_t obstacleId, ObstacleRole obstacleRole, std::shared_ptr<State> currentState,
                   ObstacleType obstacleType, double vMax, double aMax, double aMaxLong, double aMinLong,
                   std::optional<double> reactionTime, Obstacle::state_map_t trajectoryPrediction, double length,
                   double width, const std::vector<vertex> &fov)
    : Obstacle{obstacleId,
               obstacleRole,
               std::move(currentState),
               obstacleType,
               ActuatorParameters{vMax, aMax, aMaxLong, aMinLong, aMinLong},
               SensorParameters{250.0, 250.0, reactionTime},
               std::move(trajectoryPrediction),
               std::make_unique<Rectangle>(length, width),
               fov} {}

Obstacle::Obstacle(size_t obstacleId, ObstacleRole obstacleRole, std::shared_ptr<State> currentState,
                   ObstacleType obstacleType, ActuatorParameters actuatorParameters, SensorParameters sensorParameters,
                   Obstacle::state_map_t trajectoryPrediction, std::unique_ptr<Shape> shape,
                   const std::vector<vertex> &fov)
    : obstacleId(obstacleId), obstacleRole(obstacleRole), currentState(std::move(currentState)),
      obstacleType(obstacleType), actuatorParameters(actuatorParameters), sensorParameters(sensorParameters),
      trajectoryPrediction(std::move(trajectoryPrediction)), geoShape(std::move(shape)) {
    if (obstacleRole == ObstacleRole::STATIC)
        setIsStatic(true);

    if (fov.empty()) {
        // TODO update default fov values
        std::vector<vertex> fovVertices{{0.0, -400.0},        {282.8427, -282.8427},  {400.0, 0.0},
                                        {282.8427, 282.8427}, {0.0, 400.0},           {-282.8427, 282.8427},
                                        {-400.0, 0.0},        {-282.8427, -282.8427}, {0.0, -400.0}};
        setFov(geometric_operations::rotateAndTranslateVertices(
            fovVertices, {this->currentState->getXPosition(), this->currentState->getYPosition()}, 0));
    }
}

void Obstacle::setId(const size_t oId) { obstacleId = oId; }

void Obstacle::setIsStatic(bool staticObstacle) {
    if (staticObstacle) {
        actuatorParameters = ActuatorParameters::staticDefaults();
    }
}

void Obstacle::setCurrentState(const std::shared_ptr<State> &state) { currentState = state; }

void Obstacle::setObstacleType(ObstacleType type) { obstacleType = type; }

void Obstacle::setActuatorParameters(ActuatorParameters params) { actuatorParameters = params; }

void Obstacle::setSensorParameters(SensorParameters params) { sensorParameters = params; }

void Obstacle::setTrajectoryPrediction(const Obstacle::state_map_t &trajPrediction) {
    trajectoryPrediction = trajPrediction;
}

void Obstacle::setRectangleShape(double length, double width) { geoShape = std::make_unique<Rectangle>(length, width); }

void Obstacle::setGeoShape(std::unique_ptr<Shape> shape) { geoShape = std::move(shape); }

void Obstacle::appendStateToTrajectoryPrediction(const std::shared_ptr<State> &state) {
    trajectoryPrediction.insert(std::pair<size_t, std::shared_ptr<State>>(state->getTimeStep(), state));
}

void Obstacle::appendStateToHistory(const std::shared_ptr<State> &state) {
    trajectoryHistory.insert(std::pair<size_t, std::shared_ptr<State>>(state->getTimeStep(), state));
}

size_t Obstacle::getId() const { return obstacleId; }

const std::shared_ptr<State> &Obstacle::getCurrentState() const { return currentState; }

bool Obstacle::timeStepExists(size_t timeStep) {
    return (isStatic() or trajectoryPrediction.count(timeStep) == 1 or currentState->getTimeStep() == timeStep or
            trajectoryHistory.count(timeStep) == 1);
}

std::shared_ptr<State> Obstacle::getStateByTimeStep(size_t timeStep) const {
    if (isStatic())
        return currentState;
    else if (trajectoryPrediction.count(timeStep) == 1)
        return trajectoryPrediction.at(timeStep);
    else if (currentState->getTimeStep() == timeStep)
        return currentState;
    else if (trajectoryHistory.count(timeStep) == 1)
        return trajectoryHistory.at(timeStep);
    else
        throw std::logic_error("Time step does not exist. Obstacle ID: " + std::to_string(this->getId()) +
                               " - Time step: " + std::to_string(timeStep));
}

std::shared_ptr<SignalState> Obstacle::getSignalStateByTimeStep(size_t timeStep) const {
    if (isStatic())
        return currentSignalState;
    else if (signalSeries.count(timeStep) == 1)
        return signalSeries.at(timeStep);
    else if (currentSignalState && currentSignalState->getTimeStep() == timeStep)
        return currentSignalState;
    else if (signalSeriesHistory.count(timeStep) == 1)
        return signalSeriesHistory.at(timeStep);
    else
        throw std::logic_error("SignalState for time step does not exist. Obstacle ID: " +
                               std::to_string(this->getId()) + " - Time step: " + std::to_string(timeStep));
}

ObstacleType Obstacle::getObstacleType() const { return obstacleType; }

ObstacleRole Obstacle::getObstacleRole() const { return obstacleRole; }

double Obstacle::getVmax() const {
    assert(actuatorParameters);
    return actuatorParameters->getVmax();
}

double Obstacle::getAmax() const {
    assert(actuatorParameters);
    return actuatorParameters->getAmax();
}

double Obstacle::getAmaxLong() const {
    assert(actuatorParameters);
    return actuatorParameters->getAmaxLong();
}

double Obstacle::getAminLong() const {
    assert(actuatorParameters);
    return actuatorParameters->getAminLong();
}

std::optional<double> Obstacle::getReactionTime() const {
    assert(sensorParameters);
    return sensorParameters->getReactionTime();
}

std::vector<size_t> Obstacle::getPredictionTimeSteps() {
    std::vector<size_t> timeSteps;
    boost::copy(trajectoryPrediction | boost::adaptors::map_keys, std::back_inserter(timeSteps));
    return timeSteps;
}

std::vector<size_t> Obstacle::getHistoryTimeSteps() {
    std::vector<size_t> timeSteps;
    boost::copy(trajectoryHistory | boost::adaptors::map_keys, std::back_inserter(timeSteps));
    return timeSteps;
}

std::vector<size_t> Obstacle::getTimeSteps() {
    std::vector<size_t> timeSteps{getPredictionTimeSteps()};
    if (timeSteps.front() != currentState->getTimeStep())
        timeSteps.insert(timeSteps.begin(), currentState->getTimeStep());
    return timeSteps;
}

Obstacle::state_map_t Obstacle::getTrajectoryPrediction() const { return trajectoryPrediction; }

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
    } else {
        throw std::runtime_error{"obstacle shapes other than rectangle not supported by setOccupancyPolygonShape"};
    }
    shapeAtTimeStep[timeStep] = polygonShape;
    return polygonShape;
}

Shape &Obstacle::getGeoShape() { return *geoShape; }

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
}

std::vector<std::shared_ptr<Lanelet>>
Obstacle::getOccupiedLaneletsDrivingDirectionByShape(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    return setOccupiedLaneletsDrivingDirectionByShape(roadNetwork, timeStep);
}

double Obstacle::frontS(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    double lonPosition = getLonPosition(roadNetwork, timeStep);
    double theta = getCurvilinearOrientation(roadNetwork, timeStep);
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);

    // use maximum of all corners
    return lonPosition + rotatedMaximumLongitude(rect, theta);
}

void Obstacle::convertPointToCurvilinear(size_t timeStep, const std::shared_ptr<Lane> &refLane) {
    Eigen::Vector2d convertedPoint;
    convertedPoint = refLane->getCurvilinearCoordinateSystem()->convertToCurvilinearCoords(
        getStateByTimeStep(timeStep)->getXPosition(), getStateByTimeStep(timeStep)->getYPosition());
    double theta = geometric_operations::subtractOrientations(
        getStateByTimeStep(timeStep)->getGlobalOrientation(),
        refLane->getOrientationAtPosition(getStateByTimeStep(timeStep)->getXPosition(),
                                          getStateByTimeStep(timeStep)->getYPosition()));
    convertedPositions[timeStep][refLane->getContainedLaneletIDs()] = {convertedPoint.x() - 3, convertedPoint.y(),
                                                                       theta}; // TODO parameter
}

double Obstacle::frontS(size_t timeStep, const std::shared_ptr<Lane> &refLane) {
    if (convertedPositions.count(timeStep) != 1 ||
        convertedPositions[timeStep].count(refLane->getContainedLaneletIDs()) != 1) {
        try {
            convertPointToCurvilinear(timeStep, refLane);
        } catch (...) {
            std::string refInfo;
            for (const auto &ref : refLane->getCurvilinearCoordinateSystem()->referencePath())
                refInfo += "{" + std::to_string(ref.x()) + ", " + std::to_string(ref.y()) + "}, ";
            throw std::runtime_error(
                "Obstacle::frontS Custom CCS - Curvilinear Projection Error - Obstacle ID: " +
                std::to_string(obstacleId) + " - Time Step: " + std::to_string(timeStep) + " - Reference Lane: " +
                refInfo + " - x-position: " + std::to_string(trajectoryPrediction.at(timeStep)->getXPosition()) +
                " - y-position: " + std::to_string(trajectoryPrediction.at(timeStep)->getYPosition()));
        }
    }
    double lonPosition = convertedPositions[timeStep][refLane->getContainedLaneletIDs()][0];
    double theta = convertedPositions[timeStep][refLane->getContainedLaneletIDs()][2];
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);

    // use maximum of all corners
    return lonPosition + rotatedMaximumLongitude(rect, theta);
}

double Obstacle::rearS(size_t timeStep, const std::shared_ptr<Lane> &refLane) {
    if (convertedPositions.count(timeStep) != 1 or
        convertedPositions[timeStep].count(refLane->getContainedLaneletIDs()) != 1) {
        try {
            convertPointToCurvilinear(timeStep, refLane);
        } catch (...) {
            std::string refInfo;
            for (const auto &ref : refLane->getCurvilinearCoordinateSystem()->referencePath())
                refInfo += "{" + std::to_string(ref.x()) + ", " + std::to_string(ref.y()) + "}, ";
            throw std::runtime_error(
                "Obstacle::rearS Custom CCS - Curvilinear Projection Error - Obstacle ID: " +
                std::to_string(obstacleId) + " - Time Step: " + std::to_string(timeStep) + " - Reference Lane: " +
                refInfo + " - x-position: " + std::to_string(trajectoryPrediction.at(timeStep)->getXPosition()) +
                " - y-position: " + std::to_string(trajectoryPrediction.at(timeStep)->getYPosition()));
        }
    }
    double lonPosition = convertedPositions[timeStep][refLane->getContainedLaneletIDs()][0];
    double theta = convertedPositions[timeStep][refLane->getContainedLaneletIDs()][2];
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);

    // use minimum of all corners
    return lonPosition + rotatedMinimumLongitude(rect, theta);
}

double Obstacle::rightD(size_t timeStep, const std::shared_ptr<Lane> &refLane) {
    if (convertedPositions.count(timeStep) != 1 ||
        convertedPositions[timeStep].count(refLane->getContainedLaneletIDs()) != 1) {
        try {
            convertPointToCurvilinear(timeStep, refLane);
        } catch (...) {
            std::string refInfo;
            for (const auto &ref : refLane->getCurvilinearCoordinateSystem()->referencePath())
                refInfo += "{" + std::to_string(ref.x()) + ", " + std::to_string(ref.y()) + "}, ";
            throw std::runtime_error(
                "Obstacle::rightD Custom CCS - Curvilinear Projection Error - Obstacle ID: " +
                std::to_string(obstacleId) + " - Time Step: " + std::to_string(timeStep) + " - Reference Lane: " +
                refInfo + " - x-position: " + std::to_string(trajectoryPrediction.at(timeStep)->getXPosition()) +
                " - y-position: " + std::to_string(trajectoryPrediction.at(timeStep)->getYPosition()));
        }
    }
    double latPosition = convertedPositions[timeStep][refLane->getContainedLaneletIDs()][1];
    double theta = convertedPositions[timeStep][refLane->getContainedLaneletIDs()][2];
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);

    return latPosition + rotatedMinimumLatitude(rect, theta);
}

double Obstacle::leftD(size_t timeStep, const std::shared_ptr<Lane> &refLane) {
    if (convertedPositions.count(timeStep) != 1 ||
        convertedPositions[timeStep].count(refLane->getContainedLaneletIDs()) != 1) {
        try {
            convertPointToCurvilinear(timeStep, refLane);
        } catch (...) {
            std::string refInfo;
            for (const auto &ref : refLane->getCurvilinearCoordinateSystem()->referencePath())
                refInfo += "{" + std::to_string(ref.x()) + ", " + std::to_string(ref.y()) + "}, ";
            throw std::runtime_error(
                "Obstacle::leftD Custom CCS - Curvilinear Projection Error - Obstacle ID: " +
                std::to_string(obstacleId) + " - Time Step: " + std::to_string(timeStep) + " - Reference Lane: " +
                refInfo + " - x-position: " + std::to_string(trajectoryPrediction.at(timeStep)->getXPosition()) +
                " - y-position: " + std::to_string(trajectoryPrediction.at(timeStep)->getYPosition()));
        }
    }
    double latPosition = convertedPositions[timeStep][refLane->getContainedLaneletIDs()][1];
    double theta = convertedPositions[timeStep][refLane->getContainedLaneletIDs()][2];
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);

    return latPosition + rotatedMaximumLatitude(rect, theta);
}

double Obstacle::rearS(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    double lonPosition = getLonPosition(roadNetwork, timeStep);
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);
    double theta = getCurvilinearOrientation(roadNetwork, timeStep);

    // use minimum of all corners
    return lonPosition + rotatedMinimumLongitude(rect, theta);
}

double Obstacle::rightD(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    double latPos = getLatPosition(roadNetwork, timeStep);
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);
    double theta = getCurvilinearOrientation(roadNetwork, timeStep);

    return latPos + rotatedMinimumLatitude(rect, theta);
}

double Obstacle::leftD(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    double latPos = getLatPosition(roadNetwork, timeStep);
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);
    double theta = getCurvilinearOrientation(roadNetwork, timeStep);

    return latPos + rotatedMaximumLatitude(rect, theta);
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
    if (convertedPositions.count(timeStep) != 1 ||
        convertedPositions[timeStep].count(refLane->getContainedLaneletIDs()) != 1) {
        try {
            convertPointToCurvilinear(timeStep, refLane);
        } catch (...) {
            std::string refInfo;
            for (const auto &ref : refLane->getCurvilinearCoordinateSystem()->referencePath())
                refInfo += "{" + std::to_string(ref.x()) + ", " + std::to_string(ref.y()) + "}, ";
            throw std::runtime_error(
                "Obstacle::getLonPosition Custom CCS - Curvilinear Projection Error - Obstacle ID: " +
                std::to_string(obstacleId) + " - Time Step: " + std::to_string(timeStep) + " - Reference Lane: " +
                refInfo + " - x-position: " + std::to_string(trajectoryPrediction.at(timeStep)->getXPosition()) +
                " - y-position: " + std::to_string(trajectoryPrediction.at(timeStep)->getYPosition()));
        }
    }
    return convertedPositions[timeStep][refLane->getContainedLaneletIDs()][0];
}

double Obstacle::getLatPosition(size_t timeStep, const std::shared_ptr<Lane> &refLane) {
    if (convertedPositions.count(timeStep) != 1 ||
        convertedPositions[timeStep].count(refLane->getContainedLaneletIDs()) != 1) {
        try {
            convertPointToCurvilinear(timeStep, refLane);
        } catch (...) {
            std::string refInfo;
            for (const auto &ref : refLane->getCurvilinearCoordinateSystem()->referencePath())
                refInfo += "{" + std::to_string(ref.x()) + ", " + std::to_string(ref.y()) + "}, ";
            throw std::runtime_error(
                "Obstacle::getLatPosition Custom CCS - Curvilinear Projection Error - Obstacle ID: " +
                std::to_string(obstacleId) + " - Time Step: " + std::to_string(timeStep) + " - Reference Lane: " +
                refInfo + " - x-position: " + std::to_string(trajectoryPrediction.at(timeStep)->getXPosition()) +
                " - y-position: " + std::to_string(trajectoryPrediction.at(timeStep)->getYPosition()));
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
    if (convertedPositions.count(timeStep) != 1 ||
        convertedPositions[timeStep].count(refLane->getContainedLaneletIDs()) != 1) {
        try {
            convertPointToCurvilinear(timeStep, refLane);
        } catch (...) {
            std::string refInfo;
            for (const auto &ref : refLane->getCurvilinearCoordinateSystem()->referencePath())
                refInfo += "{" + std::to_string(ref.x()) + ", " + std::to_string(ref.y()) + "}, ";
            throw std::runtime_error(
                "Obstacle::getCurvilinearOrientation Custom CCS - Curvilinear Projection Error - Obstacle ID: " +
                std::to_string(obstacleId) + " - Time Step: " + std::to_string(timeStep) +
                " - Reference Lane: " + refInfo);
        }
    }
    return convertedPositions[timeStep][refLane->getContainedLaneletIDs()][2];
}

size_t Obstacle::getFirstTrajectoryTimeStep() const {
    // NOTE: trajectoryPrediction is an unordered_map, so the first element (.begin()) is not
    // necessarily the minimum element!
    if (!trajectoryPrediction.empty())
        return std::min_element(trajectoryPrediction.begin(), trajectoryPrediction.end())->first;
    else
        throw std::runtime_error("Obstacle::getFirstTrajectoryTimeStep: Obstacle with ID " +
                                 std::to_string(obstacleId) + " has not trajectoryPrediction");
}

size_t Obstacle::getLastTrajectoryTimeStep() const {
    return std::max_element(trajectoryPrediction.begin(), trajectoryPrediction.end())->first;
}

std::shared_ptr<Lane> Obstacle::getReferenceLane(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    return setReferenceLane(roadNetwork, timeStep);
}

std::vector<std::shared_ptr<Lane>> Obstacle::computeMainRef(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                            size_t timeStep,
                                                            const std::vector<std::shared_ptr<Lane>> &lanes) {
    std::vector<std::shared_ptr<Lane>> relevantOccupiedLanes;
    // use currently occupied lanelets in driving direction as lane candidates, already return if lane contains lanelet
    // of initial and final time step
    bool startEnd{false};
    bool startOrEnd{false};
    for (const auto &lane : lanes)
        if (lane->contains(getOccupiedLaneletsDrivingDirectionByShape(roadNetwork, currentState->getTimeStep())) and
            lane->contains(getOccupiedLaneletsDrivingDirectionByShape(roadNetwork, getLastTrajectoryTimeStep()))) {
            if (startEnd)
                relevantOccupiedLanes.push_back(lane);
            else {
                startEnd = true;
                relevantOccupiedLanes = {lane};
            }
            continue;
        } else if (!startEnd and lane->contains(getOccupiedLaneletsDrivingDirectionByShape(
                                     roadNetwork, getLastTrajectoryTimeStep()))) {
            if (startOrEnd)
                relevantOccupiedLanes.push_back(lane);
            else {
                startOrEnd = true;
                relevantOccupiedLanes = {lane};
            }
            continue;
        } else if (!startEnd and !startOrEnd)
            relevantOccupiedLanes.push_back(lane);

    if (relevantOccupiedLanes.size() > 1) { // iterate over all time steps starting from current time step and count
                                            // occupied lanelets in driving direction which are part of lanes
        std::map<size_t, size_t> numOccupancies;
        for (size_t newTimeStep{timeStep}; newTimeStep <= getLastTrajectoryTimeStep(); ++newTimeStep)
            for (const auto &lane : relevantOccupiedLanes)
                if (lane->contains(getOccupiedLaneletsDrivingDirectionByShape(roadNetwork, newTimeStep)))
                    numOccupancies[lane->getId()]++;

        if (!numOccupancies.empty()) { // find lane with most occupancies
            relevantOccupiedLanes = {*std::find_if(relevantOccupiedLanes.begin(), relevantOccupiedLanes.end(),
                                                   [numOccupancies](const std::shared_ptr<Lane> &lane) {
                                                       return lane->getId() == numOccupancies.rbegin()->first;
                                                   })};
        }
    }
    return relevantOccupiedLanes;
}

std::shared_ptr<Lane> Obstacle::setReferenceLane(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    if (referenceLane.count(timeStep) == 1 and referenceLane.at(timeStep) != nullptr)
        return referenceLane.at(timeStep);

    std::vector<std::shared_ptr<Lane>> relevantOccupiedLanes;
    auto refLaneTmp{computeMainRef(roadNetwork, timeStep, getOccupiedLanes(roadNetwork, timeStep))};

    if (refLaneTmp.empty()) {
        std::vector<std::shared_ptr<Lanelet>> relevantLanelets;
        for (const auto &letBase : getOccupiedLaneletsByShape(roadNetwork, timeStep))
            for (const auto &adj : lanelet_operations::adjacentLanelets(letBase, false))
                relevantLanelets.push_back(adj);
        auto lanes{lanelet_operations::createLanesBySingleLanelets(relevantLanelets, roadNetwork, fieldOfViewRear,
                                                                   fieldOfViewFront)};
        refLaneTmp = computeMainRef(roadNetwork, timeStep, lanes);
    }
    if (!refLaneTmp.empty())
        referenceLane[timeStep] = refLaneTmp.at(0);

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
        for (const auto &ref : curReferenceLane->getCurvilinearCoordinateSystem()->referencePath())
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

void Obstacle::setOccupiedLanes(const std::vector<std::shared_ptr<Lane>> &lanes, size_t timeStep) {
    if (occupiedLanes.count(timeStep) == 0)
        occupiedLanes[timeStep] = lanes;
}

void Obstacle::setObstacleRole(ObstacleRole type) { obstacleRole = type; }

void Obstacle::setOccupiedLanes(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    auto lanelets{getOccupiedLaneletsDrivingDirectionByShape(roadNetwork, timeStep)};
    assert(sensorParameters);
    std::vector<std::shared_ptr<Lane>> occLanes{lanelet_operations::createLanesBySingleLanelets(
        lanelets, roadNetwork, sensorParameters->getFieldOfViewRear(), sensorParameters->getFieldOfViewFront())};
    occupiedLanes[timeStep] = occLanes;
}

std::vector<std::shared_ptr<Lane>> Obstacle::getDrivingPathLanes(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                                 size_t timeStep) {
    auto occLanes{getOccupiedLanes(roadNetwork, timeStep)};
    if (occLanes.size() == 1)
        return occLanes;
    else {
        std::vector<std::shared_ptr<Lane>> relevantLanes;
        auto occLanelets{getOccupiedLaneletsDrivingDirectionByShape(roadNetwork, timeStep)};
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
    auto lanes{lanelet_operations::createLanesBySingleLanelets(
        lanelets, roadNetwork, sensorParameters->getFieldOfViewRear(), sensorParameters->getFieldOfViewFront())};
    setOccupiedLanes(lanes, timeStamp);
    if (!isStatic()) {
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
    if (!isStatic())
        for (const auto &timeStep : getPredictionTimeSteps())
            if (!getStateByTimeStep(timeStep)->getValidStates().lonPosition)
                convertPointToCurvilinear(roadNetwork, timeStep);
}

const polygon_type &Obstacle::getFov() const { return fov; }

void Obstacle::setFov(const std::vector<vertex> &fovVertices) {
    polygon_type polygon;
    polygon.outer().resize(fovVertices.size());
    size_t idx{0};
    for (const auto &left : fovVertices) {
        polygon.outer()[idx] = point_type{left.x, left.y};
        idx++;
    }

    fov = polygon;
    boost::geometry::simplify(polygon, fov, 0.01);
    boost::geometry::unique(fov);
    boost::geometry::correct(fov);
}

std::vector<std::shared_ptr<Lanelet>>
Obstacle::setOccupiedLaneletsDrivingDirectionByShape(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                     time_step_t timeStep) {
    if (occupiedLaneletsDrivingDir.find(timeStep) != occupiedLaneletsDrivingDir.end())
        return occupiedLaneletsDrivingDir[timeStep];

    // find all paths for all occupied initial lanelets to all occupied final lanelets
    std::set<size_t> relevantLanelets;
    for (const auto &initialLet : setOccupiedLaneletsByShape(roadNetwork, currentState->getTimeStep()))
        for (const auto &finalLet : setOccupiedLaneletsByShape(roadNetwork, getLastTrajectoryTimeStep())) {
            auto path{roadNetwork->getTopologicalMap()->findPaths(initialLet->getId(), finalLet->getId(), true)};
            relevantLanelets.insert(path.begin(), path.end());
        }

    // get lanelet objects for relevant lanelets and also consider adjacent lanelets
    for (const auto &letBase : relevantLanelets)
        for (const auto &adj : lanelet_operations::adjacentLanelets(roadNetwork->findLaneletById(letBase), true))
            relevantLanelets.insert(adj->getId());

    // todo part above could be extracted and only computed on demand (usually when trajectoryPrediction changes)
    // restrict lanelets to the ones which are currently occupied
    std::vector<std::shared_ptr<Lanelet>> lanelets;
    for (const auto &letBase : setOccupiedLaneletsByShape(roadNetwork, timeStep))
        if (relevantLanelets.find(letBase->getId()) != relevantLanelets.end())
            lanelets.push_back(letBase);

    occupiedLaneletsDrivingDir[timeStep] = lanelets;
    return occupiedLaneletsDrivingDir[timeStep];
}

void Obstacle::setCurrentSignalState(const std::shared_ptr<SignalState> &state) { currentSignalState = state; }

void Obstacle::appendSignalStateToSeries(const std::shared_ptr<SignalState> &state) {
    signalSeries.insert(std::pair<size_t, std::shared_ptr<SignalState>>(state->getTimeStep(), state));
}

void Obstacle::appendSignalStateToHistory(const std::shared_ptr<SignalState> &state) {
    signalSeriesHistory.insert(std::pair<size_t, std::shared_ptr<SignalState>>(state->getTimeStep(), state));
}
const Obstacle::signal_state_map_t &Obstacle::getSignalSeries() const { return signalSeries; }

const Obstacle::signal_state_map_t &Obstacle::getSignalSeriesHistory() const { return signalSeriesHistory; }

const std::shared_ptr<SignalState> &Obstacle::getCurrentSignalState() const { return currentSignalState; }

bool Obstacle::isStatic() const { return obstacleRole == ObstacleRole::STATIC; }

double Obstacle::drivenTrajectoryDistance() const {
    std::vector<vertex> polyline{{currentState->getXPosition(), currentState->getYPosition()}};
    for (const auto &state : trajectoryAsVector())
        polyline.push_back({state->getXPosition(), state->getYPosition()});
    return geometric_operations::computePathLengthFromPolyline(polyline).back();
}
std::vector<std::shared_ptr<State>> Obstacle::trajectoryAsVector() const {
    std::vector<std::shared_ptr<State>> trajectory;
    for (size_t timeStep{getFirstTrajectoryTimeStep()}; timeStep <= getLastTrajectoryTimeStep(); ++timeStep)
        if (trajectoryPrediction.find(timeStep) != trajectoryPrediction.end())
            trajectory.push_back(trajectoryPrediction.at(timeStep));
    return trajectory;
}

double Obstacle::getFieldOfViewRearDistance() const { return fieldOfViewRear; }

double Obstacle::getFieldOfViewFrontDistance() const { return fieldOfViewFront; }

void Obstacle::setFieldOfViewRearDistance(double distance) { fieldOfViewRear = distance; }

void Obstacle::setFieldOfViewFrontDistance(double distance) { fieldOfViewFront = distance; }
