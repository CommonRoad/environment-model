//
// Created by Sebastian Maierhofer on 01.11.20.
//

#include "obstacle.h"
#include "../geometry/geometric_operations.h"
#include <chrono>
#include <cmath>
#include "../road_network/road_network.h"

void Obstacle::setId(const size_t num) { id = num; }

//
//void Obstacle::addInLane(lane *l) { inLanes.emplace_back(l); }
//
//void Obstacle::updateInLane(std::vector<lane *> &lanes) {
//    inLanes = this->getInTracks(lanes);
//    this->updateInLanelets();
//}

//void Obstacle::updateInLanelets() {
//
//    std::vector<Lanelet *> laneletsOfOneLane;
//
//    for (const auto &it : inLanes) {
//        laneletsOfOneLane.clear();
//        laneletsOfOneLane = this->getInTracks(it->getAssLanelets());
//        inLanelets.insert(std::end(inLanelets), std::begin(laneletsOfOneLane), std::end(laneletsOfOneLane));
//    }
//}

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


void Obstacle::appendState(State state) {
    trajectoryPrediction.insert(std::pair<int, State>(state.getTimeStep(), state));
}

polygon_type Obstacle::getOccupancyPolygonShape(int timeStamp) {

    std::vector<vertice> boundingRectangleVertices;
    polygon_type polygonShape;
    // size_t i;

    if (this->getGeoShape().getType() == "rectangle") {

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
