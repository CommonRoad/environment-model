//
// Created by Sebastian Maierhofer on 01.11.20.
//

#include "obstacle.h"
#include "commonroad_cpp/geometry/geometric_operations.h"

#include <cmath>
#include <utility>

Obstacle::Obstacle(int id, bool isStatic, std::shared_ptr<State> currentState, ObstacleType obstacleType, double vMax,
                   double aMax, double aMaxLong, double aMinLong, double reactionTime,
                   std::map<size_t, std::shared_ptr<State>> trajectoryPrediction, double length, double width)
    : id(id), isStatic(isStatic), currentState(std::move(currentState)), obstacleType(obstacleType), vMax(vMax),
      aMax(aMax), aMaxLong(aMaxLong), aMinLong(aMinLong), reactionTime(reactionTime),
      trajectoryPrediction(std::move(trajectoryPrediction)), geoShape(Rectangle(length, width)) {
    if (isStatic)
        setIsStatic(isStatic);
}

void Obstacle::setId(const int obstacleId) { id = obstacleId; }

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

void Obstacle::setOwnLane(const std::vector<std::shared_ptr<Lane>> &possibleLanes, size_t timeStep) {
    if (ownLane != nullptr and
        ownLane->checkIntersection(getOccupancyPolygonShape(timeStep), ContainmentType::PARTIALLY_CONTAINED)) {
        return; // old reference lane is still valid
    }
    // assign new reference lane
    else {
        polygon_type polygonShape{getOccupancyPolygonShape(timeStep)};
        ownLane = RoadNetwork::findLaneByShape(possibleLanes, polygonShape);
    }
}

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

int Obstacle::getId() const { return id; }

bool Obstacle::getIsStatic() const { return isStatic; }

const std::shared_ptr<State> &Obstacle::getCurrentState() const { return currentState; }

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

std::shared_ptr<Lane> Obstacle::getOwnLane() const { return ownLane; }

void Obstacle::setReferenceLane(const std::shared_ptr<Lane> &lane) { referenceLane = lane; }

std::map<size_t, std::shared_ptr<State>> Obstacle::getTrajectoryPrediction() const { return trajectoryPrediction; }

size_t Obstacle::getTrajectoryLength() { return trajectoryPrediction.size(); }

polygon_type Obstacle::getOccupancyPolygonShape(size_t timeStep) {
    std::vector<vertex> boundingRectangleVertices;
    polygon_type polygonShape;
    std::shared_ptr<State> state{this->getStateByTimeStep(timeStep)};

    if (this->getGeoShape().getType() == ShapeType::rectangle) {
        // p are vertices of the bounding rectangle
        // vertices p represent the occupancy with vehicle dimensions (Theorem 1 in SPOT paper)
        boundingRectangleVertices = addObjectDimensions(
            std::vector<vertex>{vertex{0.0, 0.0}}, this->getGeoShape().getLength(), this->getGeoShape().getWidth());

        /*
         * rotate and translate the vertices of the occupancy set in local
         * coordinates to the object's reference position and rotation
         */
        std::vector<vertex> adjustedBoundingRectangleVertices =
            rotateAndTranslateVertices(boundingRectangleVertices, vertex{state->getXPosition(), state->getYPosition()},
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

std::vector<std::shared_ptr<Lane>> Obstacle::getOccupiedLanes(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                              size_t timeStep) {
    if (occupiedLanes.find(timeStep) != occupiedLanes.end())
        return occupiedLanes.at(timeStep);
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
    return occupied;
}

size_t Obstacle::getFirstTrajectoryTimeStep() { return trajectoryPrediction.begin()->second->getTimeStep(); }

size_t Obstacle::getLastTrajectoryTimeStep() {
    return trajectoryPrediction.begin()->second->getTimeStep() + getTrajectoryLength() - 1;
}

std::shared_ptr<Lane> Obstacle::getReferenceLane() const { return referenceLane; }

const std::vector<std::shared_ptr<Lanelet>> &Obstacle::getStraightOutgoings() const { return straightOutgoings; }

void Obstacle::setStraightOutgoings(const std::vector<std::shared_ptr<Lanelet>> &stroug) { straightOutgoings = stroug; }

const std::vector<std::shared_ptr<Lanelet>> &Obstacle::getLeftOutgoings() const { return leftOutgoings; }

void Obstacle::setLeftOutgoings(const std::vector<std::shared_ptr<Lanelet>> &leftoug) { leftOutgoings = leftoug; }

const std::vector<std::shared_ptr<Lanelet>> &Obstacle::getRightOutgoings() const { return rightOutgoings; }

void Obstacle::setRightOutgoings(const std::vector<std::shared_ptr<Lanelet>> &rightoug) { rightOutgoings = rightoug; }

const std::vector<std::shared_ptr<Lanelet>> &Obstacle::getOncomings() const { return oncomings; }

void Obstacle::setOncomings(const std::vector<std::shared_ptr<Lanelet>> &onc) { oncomings = onc; }

void Obstacle::convertPointToCurvilinear(size_t timeStep) const {

    Eigen::Vector2d convertedPoint = referenceLane->getCurvilinearCoordinateSystem().convertToCurvilinearCoords(
        getStateByTimeStep(timeStep)->getXPosition(), getStateByTimeStep(timeStep)->getYPosition());
    getStateByTimeStep(timeStep)->setLonPosition(convertedPoint.x());
    getStateByTimeStep(timeStep)->setLatPosition(convertedPoint.y());
    double theta = getStateByTimeStep(timeStep)->getGlobalOrientation() -
                   getReferenceLane()->getLanelet().getOrientationAtPosition(
                       getStateByTimeStep(timeStep)->getXPosition(), getStateByTimeStep(timeStep)->getYPosition());
    getStateByTimeStep(timeStep)->setCurvilinearOrientation(theta);
}
