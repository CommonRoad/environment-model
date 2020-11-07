#include "obstacle.h"
#include "../geometry/geometricOperations.h"
//#include "../lanelets/lanelet_operations.h"
#include <chrono>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


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


//void Obstacle::useShapeAsRef(const bool val) { useShape = val; }

size_t Obstacle::getId() const { return id; }


//const std::vector<lane *> &Obstacle::getInLane() const { return inLanes; }

//const std::vector<std::vector<occTypes>> &Obstacle::getOccupancyMatrix() const { return occupancyMatrix; }

//const std::vector<std::vector<occTypes>> *Obstacle::getOccupancyMatrixPtr() { return &occupancyMatrix; }

//void Obstacle::setOccupancyMatrix(const std::vector<std::vector<occTypes>> &occMatrix) { occupancyMatrix = occMatrix; }

//void Obstacle::setOccupancyMatrix(const std::vector<std::vector<occTypes>> &&occMatrix) { occupancyMatrix = occMatrix; }


void Obstacle::setVmax(const double vmax) { v_max = isStatic ? 0.0 : vmax; }

void Obstacle::setAmax(const double amax) { a_max = isStatic ? 0.0 : amax; }

void Obstacle::setAmaxLong(const double amax_long) { a_max_long = isStatic ? 0.0 : amax_long; }

void Obstacle::setAminLong(const double amin_long) { a_min_long = isStatic ? 0.0 : amin_long; }

//void Obstacle::setReachableLanes(std::vector<lane *> lanes) {
//    reachableLanes.clear();
//    for (size_t i = 0; i < lanes.size(); i++) {
//        reachableLanes.emplace_back(lanes[i]->getId());
//    }
//}


double Obstacle::getVmax() const { return v_max; }

double Obstacle::getAmax() const { return a_max; }

double Obstacle::getAmaxLong() const { return a_max_long; }

double Obstacle::getAminLong() const { return a_min_long; }

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

void Obstacle::setIsStatic(bool st) {
    isStatic = st;
    if (st) {
        v_max = 0.0;
        a_max = 0.0;
        a_min_long = 0.0;
        a_max_long = 0.0;
    }
}

void Obstacle::appendState(State state) {
    trajectoryPrediction.insert(std::pair<int, State>(state.getTimeStep(), state));
}

bool Obstacle::getIsStatic() const { return isStatic; }

//bool Obstacle::findLaneletsCorrespondingToObstacle(const std::vector<vehicularLanelet *> &intersectionLanelets,
//                                                   std::vector<vehicularLanelet *> &inLanelet) {
//
//    size_t i;
//    std::vector<vehicularLanelet *> oppositeLanelets;
//
//    double laneletOrientation; // orientation of lanelet at Obstacle position
//    double obstacleOrientation = this->getOrientation();
//    // if Obstacle intersects with any lanelet --> Obstacle is in road network
//
//    vertice pos = vertice{this->getXpos(), this->getYpos()};
//    std::pair<double, double> validOrientations = config::getValidTrackOrientations();
//    for (i = 0; i < intersectionLanelets.size(); i++) {
//        // calculate orientation of lanelet at Obstacle position
//        laneletOrientation = calcAngleOfVerticesAtPosition((intersectionLanelets)[i]->getCenterVertices(), pos);
//
//        auto relationToLanelet =
//            orientationToTrack(obstacleOrientation, laneletOrientation, this->getOrientationError(), validOrientations);
//        // Obstacle is only in a lanelet if it has approx. the same orientation
//        if (relationToLanelet) {
//            double orientationToLanelet = relationToLanelet.value();
//            if (orientationToLanelet < 0) {
//                oppositeLanelets.emplace_back(intersectionLanelets[i]);
//            } else {
//                inLanelet.emplace_back(intersectionLanelets[i]);
//            }
//        }
//    }
//
//    if (inLanelet.empty()) {
//        inLanelet = std::move(oppositeLanelets);
//        return true; // lanelets with opposite direction
//    }
//    return false;
//}

shape &Obstacle::getGeoShape() { return geoShape; }

Obstacle::Obstacle(const bool isStatic) {
//        occupancyMatrix = std::vector<std::vector<occTypes>>{};
//        inLanes = std::vector<lane *>{};
//
//        reachableLanes = std::vector<size_t>{};
    setIsStatic(isStatic);
}

ObstacleType Obstacle::getType() const {
    return type;
}

void Obstacle::setType(ObstacleType type) {
    Obstacle::type = type;
}

const State &Obstacle::getCurrentState() const {
    return currentState;
}

void Obstacle::setCurrentState(const State &currentState) {
    Obstacle::currentState = currentState;
}

ObstacleType matchObstacleTypeToString(const char *type){
    if (!(strcmp(type, "car")))
        return ObstacleType::car;
    else if (!(strcmp(type, "truck")))
        return ObstacleType::truck;
    else if (!(strcmp(type, "pedestrian")))
        return ObstacleType::pedestrian;
    else if (!(strcmp(type, "bus")))
        return ObstacleType::bus;
    else
        return ObstacleType::unknown;
}
//
//std::vector<Lanelet> Obstacle::tracksAtObjPositionOrShape(const std::vector<Lanelet> &lanelets) {
//    std::vector<Lanelet> trackCandidates;
//
//    // get lanelets which intersect with shape of ego vehicle
//    // polygon_type polygonShape = this->getOccupancyPolygonShape();
//    trackCandidates = findTracksByShape(lanelets, polygonShape);
//
//    return trackCandidates;
//}
//
//std::vector<Lanelet *> Obstacle::getInLanelets(const std::vector<Lanelet> &lanelets, int timeStep) {
//    std::vector<Lanelet> tracksAtObjPosition = this->tracksAtObjPositionOrShape(lanelets);
//    if (currentState.getTimeStep() == timeStep){
//        vertice objPos{currentState.getXPosition(), currentState.getYPosition()};
//        std::vector<std::optional<double>> tracksOrientation(tracksAtObjPosition.size());
//        size_t i;
//        std::vector<Lanelet> inLanelets;
//
//        /*
//         * assume that a traffic participant is only in a track, if its
//         * orientation differs less than +- pi/5 from the track orientation
//         */
//        //const auto validOrientations = config::getValidTrackOrientations();
//        //double trackOrientation;
//
//        for (i = 0; i < tracksAtObjPosition.size(); i++) {
//
//            trackOrientation = calcAngleOfVerticesAtPosition(tracksAtObjPosition[i]->getCenterVertices(), objPos);
//            tracksOrientation[i] = orientationToTrack(this->getOrientation(), trackOrientation, this->getOrientationError(),
//                                                      validOrientations);
//
//            if (tracksOrientation[i] && tracksOrientation[i].value() >= 0) {
//
//                inTracks.push_back(tracksAtObjPosition[i]);
//            }
//        }
//
//        // consider tracks with opposite direction, if there is no track with appropriate orientation
//        if (inTracks.size() == 0) {
//            for (i = 0; i < tracksAtObjPosition.size(); i++) {
//                if (tracksOrientation[i] && tracksOrientation[i].value() < 0) {
//                    inTracks.emplace_back(tracksAtObjPosition[i]);
//                }
//            }
//        }
//        return inTracks;
//    }
//
//
//}
