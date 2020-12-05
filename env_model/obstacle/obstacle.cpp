//
// Created by Sebastian Maierhofer on 01.11.20.
//

#include "obstacle.h"
#include "../geometry/geometric_operations.h"
#include <cmath>
#include <utility>

void Obstacle::setId(const size_t num) { id = num; }

void Obstacle::setIsStatic(bool st) {
    isStatic = st;
    if (st) {
        v_max = 0.0;
        a_max = 0.0;
        a_min_long = 0.0;
        a_max_long = 0.0;
    }
}

size_t Obstacle::getId() const { return id; }

void Obstacle::setVmax(const double vmax) { v_max = isStatic ? 0.0 : vmax; }

void Obstacle::setAmax(const double amax) { a_max = isStatic ? 0.0 : amax; }

void Obstacle::setAmaxLong(const double amax_long) { a_max_long = isStatic ? 0.0 : amax_long; }

void Obstacle::setAminLong(const double amin_long) { a_min_long = isStatic ? 0.0 : amin_long; }

double Obstacle::getVmax() const { return v_max; }

double Obstacle::getAmax() const { return a_max; }

double Obstacle::getAmaxLong() const { return a_max_long; }

double Obstacle::getAminLong() const { return a_min_long; }

Shape &Obstacle::getGeoShape() { return geoShape; }

ObstacleType Obstacle::getType() const {return type;}

void Obstacle::setType(ObstacleType ty) {type = ty;}

const State &Obstacle::getCurrentState() const {return currentState;}

void Obstacle::setCurrentState(const State &state) {currentState = state;}

bool Obstacle::getIsStatic() const { return isStatic; }

std::map<int, State> Obstacle::getTrajectoryPrediction() const { return trajectoryPrediction; }

int Obstacle::getTrajectoryLength() { return trajectoryPrediction.size(); }


void Obstacle::appendState(State state) {
    trajectoryPrediction.insert(std::pair<int, State>(state.getTimeStep(), state));
}

polygon_type Obstacle::getOccupancyPolygonShape(int timeStamp) {

    std::vector<vertice> boundingRectangleVertices;
    polygon_type polygonShape;
    // size_t i;

    if (this->getGeoShape().getType() == "Rectangle") {

        // p are vertices of the bounding rectangle
        // vertices p represent the occupancy with vehicle dimensions (Theorem 1)
        boundingRectangleVertices = addObjectDimensions(
            std::vector<vertice>{vertice{0.0, 0.0}}, this->getGeoShape().getLength(), this->getGeoShape().getWidth());

        /*
         * rotate and translate the vertices of the occupancy set in local
         * coordinates to the object's reference position and rotation
         */
        std::vector<vertice> adjustedBoundingRectangleVertices = rotateAndTranslateVertices(
            boundingRectangleVertices, vertice{this->trajectoryPrediction.at(timeStamp).getXPosition(), this->trajectoryPrediction.at(timeStamp).getYPosition()}, this->trajectoryPrediction.at(timeStamp).getOrientation());

        polygonShape.outer().resize(adjustedBoundingRectangleVertices.size() + 1);

        // make polygonshape from previously created vertices
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

std::vector<std::shared_ptr<Lanelet>> Obstacle::getOccupiedLanelets(const std::shared_ptr<RoadNetwork>& roadNetwork, int timeStep) {
    if(occupiedLanelets.find(timeStep) != occupiedLanelets.end())
        return occupiedLanelets.at(timeStep);
    polygon_type polygonShape{getOccupancyPolygonShape(timeStep)};
    std::vector<std::shared_ptr<Lanelet>> occupied{roadNetwork->findOccupiedLaneletsByShape(polygonShape)};
    occupiedLanelets.insert(std::pair<int, std::vector<std::shared_ptr<Lanelet>>>(timeStep, occupied));

    return occupied;
}

void Obstacle::setLane(std::vector<std::shared_ptr<Lane>> lanes, int timeStep) {
    if(occupiedLane.find(timeStep-1) != occupiedLane.end() and occupiedLane.at(timeStep-1)->checkIntersection(getOccupancyPolygonShape(timeStep), PARTIALLY_CONTAINED)) {
        occupiedLane.insert(std::pair<int, std::shared_ptr<Lane>>(timeStep, occupiedLane.at(timeStep - 1)));
    }
    // find new lane
    else {
        polygon_type polygonShape{getOccupancyPolygonShape(timeStep)};
        std::shared_ptr<Lane> occupied{RoadNetwork::findLaneByShape(std::move(lanes), polygonShape)};
        occupiedLane.insert(std::pair<int, std::shared_ptr<Lane>>(timeStep, occupied));
    }
}

std::shared_ptr<Lane> Obstacle::getLane(int timeStep) {return occupiedLane.at(timeStep);}


double Obstacle::frontS(int timeStep) {
    double s = getLonPosition(timeStep);
    double width = geoShape.getWidth();
    double length = geoShape.getLength();
    double theta = trajectoryPrediction.at(timeStep).getOrientation() - getLane(timeStep)->getLanelet().getOrientationAtPosition(getTrajectoryPrediction().at(timeStep).getXPosition(), getTrajectoryPrediction().at(timeStep).getYPosition());
    return std::min({(length / 2) * cos(theta) - (width / 2) * sin(theta) + s,
              (length / 2) * cos(theta) - (-width / 2) * sin(theta) + s,
              (-length / 2) * cos(theta) - (width / 2) * sin(theta) + s,
              (-length / 2) * cos(theta) - (-width / 2) * sin(theta) + s});

}

double Obstacle::rearS(int timeStep) {
    double s = getLonPosition(timeStep);
    double width = geoShape.getWidth();
    double length = geoShape.getLength();
    double theta = trajectoryPrediction.at(timeStep).getOrientation() - getLane(timeStep)->getLanelet().getOrientationAtPosition(getTrajectoryPrediction().at(timeStep).getXPosition(), getTrajectoryPrediction().at(timeStep).getYPosition());
    return std::max({(length / 2) * cos(theta) - (width / 2) * sin(theta) + s,
                     (length / 2) * cos(theta) - (-width / 2) * sin(theta) + s,
                     (-length / 2) * cos(theta) - (width / 2) * sin(theta) + s,
                     (-length / 2) * cos(theta) - (-width / 2) * sin(theta) + s});
}

double Obstacle::getLonPosition(int timeStep) {
    // Start measuring time
    if(trajectoryPrediction.at(timeStep).getValidStates().lonPosition)
        return trajectoryPrediction.at(timeStep).getLonPosition();
    trajectoryPrediction.at(timeStep).convertPointToCurvilinear(getLane(timeStep)->getCurvilinearCoordinateSystem());

    return trajectoryPrediction.at(timeStep).getLonPosition();
}

double Obstacle::getLatPosition(int timeStep) {
    if(trajectoryPrediction.at(timeStep).getValidStates().latPosition)
        return trajectoryPrediction.at(timeStep).getLatPosition();
    trajectoryPrediction.at(timeStep).convertPointToCurvilinear(getLane(timeStep)->getCurvilinearCoordinateSystem());
    return trajectoryPrediction.at(timeStep).getLatPosition();
}

