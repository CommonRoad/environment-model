//
// Created by Sebastian Maierhofer on 01.11.20.
//

#include "obstacle.h"
#include "../geometry/geometric_operations.h"
#include <cmath>
#include <utility>

Obstacle::Obstacle(int id,
                   bool isStatic,
                   const State &currentState,
                   ObstacleType obstacleType,
                   double vMax,
                   double aMax,
                   double aMaxLong,
                   double aMinLong,
                   double reactionTime,
                   std::map<int, State> trajectoryPrediction,
                   double length,
                   double width) :
                   id(id),
                   isStatic(isStatic),
                   currentState(currentState),
                   obstacleType(obstacleType),
                   vMax(vMax),
                   aMax(aMax),
                   aMaxLong(aMaxLong),
                   aMinLong(aMinLong),
                   reactionTime(reactionTime),
                   trajectoryPrediction(std::move(trajectoryPrediction)),
                   geoShape(Rectangle(length, width)) {
    if(isStatic)
        setIsStatic(isStatic);
}


void Obstacle::setId(const int num) { id = num; }

void Obstacle::setIsStatic(bool st) {
    isStatic = st;
    if (st) {
        vMax = 0.0;
        aMax = 0.0;
        aMinLong = 0.0;
        aMaxLong = 0.0;
    }
}

void Obstacle::setCurrentState(const State &state) {currentState = state;}

void Obstacle::setObstacleType(ObstacleType type) {obstacleType = type;}

void Obstacle::setVmax(const double vmax) { vMax = isStatic ? 0.0 : vmax; }

void Obstacle::setAmax(const double amax) { aMax = isStatic ? 0.0 : amax; }

void Obstacle::setAmaxLong(const double amax) { aMaxLong = isStatic ? 0.0 : amax; }

void Obstacle::setAminLong(const double amin) { aMinLong = isStatic ? 0.0 : amin; }

void Obstacle::setReactionTime(const double tReact) { reactionTime = isStatic ? 0.0 : tReact; }

void Obstacle::setReferenceLane(const std::vector<std::shared_ptr<Lane>>& possibleLanes, int timeStep) {
    if(referenceLane != nullptr
    and referenceLane->checkIntersection(getOccupancyPolygonShape(timeStep),
                                         ContainmentType::PARTIALLY_CONTAINED)) {
        return;
    }
    // assign new reference lane
    else {
        polygon_type polygonShape{getOccupancyPolygonShape(timeStep)};
        referenceLane = RoadNetwork::findLaneByShape(possibleLanes, polygonShape);
    }
}

void Obstacle::setTrajectoryPrediction(const std::map<int, State> &trajPrediction) {
    trajectoryPrediction = trajPrediction;
}

void Obstacle::setRectangleShape(double length, double width) {
    geoShape = Rectangle(length, width);
}

void Obstacle::appendStateToTrajectoryPrediction(State state) {
    trajectoryPrediction.insert(std::pair<int, State>(state.getTimeStep(), state));
}

void Obstacle::appendStateToHistory(State state) {
    history.insert(std::pair<int, State>(state.getTimeStep(), state));
}

int Obstacle::getId() const { return id; }

bool Obstacle::getIsStatic() const { return isStatic; }

const State &Obstacle::getCurrentState() const {return currentState;}

State Obstacle::getStateByTimeStep(int timeStep) const {
    if(trajectoryPrediction.count(timeStep) == 1)
        return trajectoryPrediction.at(timeStep);
    else if(currentState.getTimeStep() == timeStep)
        return currentState;
    else if(history.count(timeStep) == 1)
        return history.at(timeStep);
    else
        throw std::logic_error("Time step does not exist");
}

ObstacleType Obstacle::getObstacleType() const {return obstacleType;}

double Obstacle::getVmax() const { return vMax; }

double Obstacle::getAmax() const { return aMax; }

double Obstacle::getAmaxLong() const { return aMaxLong; }

double Obstacle::getAminLong() const { return aMinLong; }

double Obstacle::getReactionTime() const { return reactionTime; }

std::shared_ptr<Lane> Obstacle::getReferenceLane() const { return referenceLane; }

std::map<int, State> Obstacle::getTrajectoryPrediction() const { return trajectoryPrediction; }

int Obstacle::getTrajectoryLength() { return trajectoryPrediction.size(); }

polygon_type Obstacle::getOccupancyPolygonShape(int timeStep) {

    std::vector<vertice> boundingRectangleVertices;
    polygon_type polygonShape;
    // size_t i;

    State state { this->getStateByTimeStep(timeStep) };

    if (this->getGeoShape().getType() == ShapeType::rectangle) {

        // p are vertices of the bounding rectangle
        // vertices p represent the occupancy with vehicle dimensions (Theorem 1)
        boundingRectangleVertices = addObjectDimensions(
            std::vector<vertice>{vertice{0.0, 0.0}}, this->getGeoShape().getLength(),
            this->getGeoShape().getWidth());

        /*
         * rotate and translate the vertices of the occupancy set in local
         * coordinates to the object's reference position and rotation
         */
        std::vector<vertice> adjustedBoundingRectangleVertices = rotateAndTranslateVertices(
            boundingRectangleVertices, vertice{state.getXPosition(),
                                               state.getYPosition()},
            state.getOrientation());

        polygonShape.outer().resize(adjustedBoundingRectangleVertices.size() + 1);

        // make polygon shape from previously created vertices
        for (size_t i = 0; i < adjustedBoundingRectangleVertices.size(); i++) {
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

std::vector<std::shared_ptr<Lanelet>> Obstacle::getOccupiedLanelets(const std::shared_ptr<RoadNetwork>& roadNetwork,
                                                                    int timeStep) {
    if(occupiedLanelets.find(timeStep) != occupiedLanelets.end())
        return occupiedLanelets.at(timeStep);
    polygon_type polygonShape{getOccupancyPolygonShape(timeStep)};
    std::vector<std::shared_ptr<Lanelet>> occupied{roadNetwork->findOccupiedLaneletsByShape(polygonShape)};
    occupiedLanelets.insert(std::pair<int, std::vector<std::shared_ptr<Lanelet>>>(timeStep, occupied));

    return occupied;
}

double Obstacle::frontS(int timeStep) {
    double s = getLonPosition(timeStep);
    double width = geoShape.getWidth();
    double length = geoShape.getLength();
    double theta = getStateByTimeStep(timeStep).getOrientation() - getReferenceLane()->
            getLanelet().getOrientationAtPosition(getStateByTimeStep(timeStep).getXPosition(),
                                                  getStateByTimeStep(timeStep).getYPosition());
    return std::max({(length / 2) * cos(theta) - (width / 2) * sin(theta) + s,
              (length / 2) * cos(theta) - (-width / 2) * sin(theta) + s,
              (-length / 2) * cos(theta) - (width / 2) * sin(theta) + s,
              (-length / 2) * cos(theta) - (-width / 2) * sin(theta) + s});
}

double Obstacle::rearS(int timeStep) {
    double s = getLonPosition(timeStep);
    double width = geoShape.getWidth();
    double length = geoShape.getLength();
    double theta = getStateByTimeStep(timeStep).getOrientation() - getReferenceLane()->
            getLanelet().getOrientationAtPosition(getStateByTimeStep(timeStep).getXPosition(),
                                                  getStateByTimeStep(timeStep).getYPosition());
    return std::min({(length / 2) * cos(theta) - (width / 2) * sin(theta) + s,
                     (length / 2) * cos(theta) - (-width / 2) * sin(theta) + s,
                     (-length / 2) * cos(theta) - (width / 2) * sin(theta) + s,
                     (-length / 2) * cos(theta) - (-width / 2) * sin(theta) + s});
}

double Obstacle::getLonPosition(int timeStep) const {
    // Start measuring time
    if(getStateByTimeStep(timeStep).getValidStates().lonPosition)
        return getStateByTimeStep(timeStep).getLonPosition();
    getStateByTimeStep(timeStep).convertPointToCurvilinear(getReferenceLane()->getCurvilinearCoordinateSystem());

    return getStateByTimeStep(timeStep).getLonPosition();
}

double Obstacle::getLatPosition(int timeStep) const {
    if(getStateByTimeStep(timeStep).getValidStates().latPosition)
        return getStateByTimeStep(timeStep).getLatPosition();
    getStateByTimeStep(timeStep).convertPointToCurvilinear(getReferenceLane()->getCurvilinearCoordinateSystem());

    return getStateByTimeStep(timeStep).getLatPosition();
}
