//
// Created by Sebastian Maierhofer on 23.10.20.
//
#include "lanelet.h"

#include <utility>
#include "boost/geometry.hpp"

namespace bg = boost::geometry;


Lanelet::Lanelet(int id,
                 std::vector<vertice> leftBorder,
                 std::vector<vertice> rightBorder,
                 std::vector<LaneletType> type,
                 std::vector<ObstacleType> oneWay,
                 std::vector<ObstacleType> userBidirectional) :
                 id{id},
                 leftBorder{std::move(leftBorder)},
                 rightBorder{std::move(rightBorder)},
                 laneletType{std::move(type)},
                 userOneWay{std::move(oneWay)},
                 userBidirectional{std::move(userBidirectional)} {
    createCenterVertices();
    constructOuterPolygon();
}

Lanelet::Lanelet(int id,
                 std::vector<vertice> leftBorder,
                 std::vector<vertice> rightBorder,
                 std::vector<std::shared_ptr<Lanelet>> predecessorLanelets,
                 std::vector<std::shared_ptr<Lanelet>> successorLanelets,
                 std::vector<LaneletType> type,
                 std::vector<ObstacleType> oneWay = std::vector<ObstacleType>(),
                 std::vector<ObstacleType> userBidirectional = std::vector<ObstacleType>()) :
                 id{id},
                 leftBorder{std::move(leftBorder)},
                 rightBorder{std::move(rightBorder)},
                 predecessorLanelets{std::move(predecessorLanelets)},
                 successorLanelets{std::move(successorLanelets)},
                 laneletType{std::move(type)},
                 userOneWay{std::move(oneWay)},
                 userBidirectional{std::move(userBidirectional)} {
    createCenterVertices();
    constructOuterPolygon();
}

void Lanelet::setId(const int num) { id = num; }

void Lanelet::setLeftAdjacent(const std::shared_ptr<Lanelet>& left, DrivingDirection dir) {
    adjacentLeft.adj = left;
    adjacentLeft.dir = dir;
}

void Lanelet::setRightAdjacent(const std::shared_ptr<Lanelet>& right, DrivingDirection dir) {
    adjacentRight.adj = right;
    adjacentRight.dir = dir;
}

void Lanelet::setLeftBorderVertices(const std::vector<vertice> &leftBorderVertices) { leftBorder = leftBorderVertices; }

void Lanelet::setRightBorderVertices(const std::vector<vertice> &rightBorderVertices) {
    rightBorder = rightBorderVertices;
}

void Lanelet::setLaneletType(const std::vector<LaneletType>& laType) { laneletType = laType; }

void Lanelet::setUserOneWay(const std::vector<ObstacleType> &user) { userOneWay = user; }

void Lanelet::setUserBidirectional(const std::vector<ObstacleType> &user) { userBidirectional = user;}

void Lanelet::setStopLine(const StopLine& sl) { stopLine = sl; }

void Lanelet::addLeftVertex(const vertice left) { leftBorder.push_back(left); }

void Lanelet::addRightVertex(const vertice right) { rightBorder.push_back(right); }

void Lanelet::addCenterVertex(const vertice center) { centerVertices.push_back(center); }

void Lanelet::addPredecessor(const std::shared_ptr<Lanelet>& pre) { predecessorLanelets.push_back(pre); }

void Lanelet::addSuccessor(const std::shared_ptr<Lanelet>& suc) { successorLanelets.push_back(suc); }

void Lanelet::addTrafficLight(const std::shared_ptr<TrafficLight>& light) { trafficLights.push_back(light); }

void Lanelet::addTrafficSign(const std::shared_ptr<TrafficSign>& sign) { trafficSigns.push_back(sign); }

int Lanelet::getId() const { return id; }

std::vector<std::shared_ptr<Lanelet>> Lanelet::getPredecessors() const { return predecessorLanelets; }

std::vector<std::shared_ptr<Lanelet>> Lanelet::getSuccessors() const { return successorLanelets; }

const std::vector<vertice> &Lanelet::getCenterVertices() const { return centerVertices; }

const std::vector<vertice> &Lanelet::getLeftBorderVertices() const { return leftBorder; }

const std::vector<vertice> &Lanelet::getRightBorderVertices() const { return rightBorder; }

std::vector<std::shared_ptr<TrafficLight>> Lanelet::getTrafficLights() const { return trafficLights; }

std::vector<std::shared_ptr<TrafficSign>> Lanelet::getTrafficSigns() const { return trafficSigns; }

const polygon_type &Lanelet::getOuterPolygon() const {return outerPolygon;}

const box &Lanelet::getBoundingBox() const {return boundingBox;}

const std::vector<LaneletType> &Lanelet::getLaneletType() const { return laneletType; }

const std::vector<ObstacleType> &Lanelet::getUserOneWay() const { return userOneWay; }

const std::vector<ObstacleType> &Lanelet::getUserBidirectional() const { return userBidirectional; }

const Lanelet::adjacent &Lanelet::getAdjacentLeft() const {return adjacentLeft;}

const Lanelet::adjacent &Lanelet::getAdjacentRight() const {return adjacentRight;}

const StopLine &Lanelet::getStopLine() const { return stopLine; }

bool Lanelet::applyIntersectionTesting(const polygon_type &polygon_shape) const {
    return bg::intersects(polygon_shape, this->getBoundingBox()) &&
           bg::intersects(polygon_shape, this->getOuterPolygon());
}

bool Lanelet::checkIntersection(const polygon_type &polygon_shape, int intersection_type) const {
    switch (intersection_type) {
        case PARTIALLY_CONTAINED: {
            return this->applyIntersectionTesting(polygon_shape);
        }
        case COMPLETELY_CONTAINED: {
            return bg::within(polygon_shape, this->getOuterPolygon());
        }
        default:
            return false;
    }
}

void Lanelet::constructOuterPolygon() {
    const std::vector<vertice> &leftBorderTemp = this->getLeftBorderVertices();
    const std::vector<vertice> &rightBorderTemp = this->getRightBorderVertices();

    if (!leftBorderTemp.empty()) {

        int idx = 0;
        polygon_type polygon;
        polygon.outer().resize(leftBorderTemp.size() + rightBorderTemp.size() + 1);

        for (auto &it : leftBorderTemp) {
            polygon.outer()[idx] = point_type{it.x, it.y};
            idx++;
        }
        for (auto &it : boost::adaptors::reverse(rightBorderTemp)) {
            polygon.outer()[idx] = point_type{it.x, it.y};
            idx++;
        }
        polygon.outer().back() = point_type{leftBorderTemp[0].x, leftBorderTemp[0].y};

        bg::simplify(polygon, outerPolygon, 0.01);
        bg::unique(outerPolygon);
        bg::correct(outerPolygon);

        bg::envelope(outerPolygon, boundingBox); // set bounding box
    }
}

void Lanelet::createCenterVertices() {
    // initialise
    int numVertices = leftBorder.size();

    for (int i = 0; i < numVertices; i++) {
        /*
         * calculate a center vertex as the arithmetic mean between the opposite
         * vertex on the left and right border
         * (calculate x and y values separately in order to minimize error)
         */
        vertice newVertex{};
        newVertex.x = 0.5 * (leftBorder[i].x + rightBorder[i].x);
        newVertex.y = 0.5 * (leftBorder[i].y + rightBorder[i].y);
        addCenterVertex(newVertex);
    }
}

double Lanelet::getOrientationAtPosition(double positionX, double positionY) {
    std::vector<double> dif(centerVertices.size()-1);
    for(int i = 0; i < centerVertices.size() - 1; ++i){
        vertice vert{centerVertices[i]};
        dif[i] = sqrt(pow(vert.x - positionX, 2) + pow(vert.y - positionY, 2));
    }
    int closestIndex{static_cast<int>(std::min_element(dif.begin(), dif.end()) - dif.begin())};
    vertice vert1{centerVertices[closestIndex]};
    vertice vert2{centerVertices[closestIndex + 1]};
    return atan2(vert2.y - vert1.y, vert2.x - vert1.x);
}
