//
// Created by Sebastian Maierhofer on 23.10.20.
//

#include "lanelet.h"
#include <utility>
#include "boost/geometry.hpp"

namespace bg = boost::geometry;

void Lanelet::setId(const size_t num) { id = num; }

size_t Lanelet::getId() const { return id; }

void Lanelet::setLeftBorderVertices(const std::vector<vertice> &leftBorderVertices) { leftBorder = leftBorderVertices; }

void Lanelet::setRightBorderVertices(const std::vector<vertice> &rightBorderVertices) {
    rightBorder = rightBorderVertices;
}

void Lanelet::setCenterVertices(const std::vector<vertice> &center) { centerVertices = center; }

// Newly added functions which potentiall make setLeftBorder,... obsolete
// Move the information instead of a copy
void Lanelet::moveLeftBorder(std::vector<vertice> &&leftBorderVertices) { leftBorder = std::move(leftBorderVertices); }

void Lanelet::moveRightBorder(std::vector<vertice> &&rightBorderVertices) {
    rightBorder = std::move(rightBorderVertices);
}

void Lanelet::moveCenterVertices(std::vector<vertice> &&center) { centerVertices = std::move(center); }

void Lanelet::addLeftVertice(const vertice left) { leftBorder.push_back(left); }

void Lanelet::addRightVertice(const vertice right) { rightBorder.push_back(right); }

void Lanelet::addCenterVertice(const vertice center) { centerVertices.push_back(center); }

void Lanelet::createCenterVertices() {
    // initialise
    size_t numVertices = leftBorder.size();

    for (size_t i = 0; i < numVertices; i++) {
        /*
         * calculate a center vertex as the arithmetic mean between the opposite
         * vertex on the left and right border
         * (calculate x and y values seperately in order to minimize error)
         */
        vertice newVertice{};
        newVertice.x = 0.5 * (leftBorder[i].x + rightBorder[i].x);
        newVertice.y = 0.5 * (leftBorder[i].y + rightBorder[i].y);
        addCenterVertice(newVertice);
    }
}

void Lanelet::addTrafficLight(const std::shared_ptr<TrafficLight>& light) { trafficLights.push_back(light); }
void Lanelet::addTrafficSign(const std::shared_ptr<TrafficSign>& sign) { trafficSigns.push_back(sign); }

std::vector<vertice> Lanelet::getLeftBorderVerticesDirect() const { return leftBorder; }
std::vector<vertice> Lanelet::getRightBorderVerticesDirect() const { return rightBorder; }
std::vector<vertice> Lanelet::getCenterVerticesDirect() const { return centerVertices; }

const std::vector<vertice> &Lanelet::getCenterVertices() const { return centerVertices; }
const std::vector<vertice> &Lanelet::getLeftBorderVertices() const { return leftBorder; }
const std::vector<vertice> &Lanelet::getRightBorderVertices() const { return rightBorder; }

std::vector<std::shared_ptr<Lanelet>> Lanelet::getPredecessors() const { return predecessorLanelets; }
std::vector<std::shared_ptr<Lanelet>> Lanelet::getSuccessors() const { return successorLanelets; }

void Lanelet::addPredecessor(const std::shared_ptr<Lanelet>& pre) { predecessorLanelets.push_back(pre); }
void Lanelet::addSuccessor(const std::shared_ptr<Lanelet>& suc) { successorLanelets.push_back(suc); }

void Lanelet::setLeftAdjacent(const std::shared_ptr<Lanelet>& left, DrivingDirection dir) {
    adjacentLeft.adj = left;
    adjacentLeft.dir = dir;
}

void Lanelet::setRightAdjacent(const std::shared_ptr<Lanelet>& right, DrivingDirection dir) {
    adjacentRight.adj = right;
    adjacentRight.dir = dir;
}

std::vector<std::shared_ptr<TrafficLight>> Lanelet::getTrafficLight() const { return trafficLights; }
std::vector<std::shared_ptr<TrafficSign>> Lanelet::getTrafficSigns() const { return trafficSigns; }

void Lanelet::constructOuterPolygon() {
    const std::vector<vertice> &leftBorderTemp = this->getLeftBorderVertices();
    const std::vector<vertice> &rightBorderTemp = this->getRightBorderVertices();

    if (!leftBorderTemp.empty()) {

        size_t idx = 0;
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

Lanelet::Lanelet() { id = 0; }

bool Lanelet::applyIntersectionTesting(const polygon_type &intersecting) const {
    return bg::intersects(intersecting, this->getBoundingBox()) &&
           bg::intersects(intersecting, this->getOuterPolygon());
}

const polygon_type &Lanelet::getOuterPolygon() const {return outerPolygon;}

void Lanelet::setOuterPolygon(const polygon_type &poly) {outerPolygon = poly;}

const box &Lanelet::getBoundingBox() const {return boundingBox;}

void Lanelet::setBoundingBox(const box &box) {
    boundingBox = box;
}

bool Lanelet::checkIntersection(const polygon_type &intersecting, size_t intersection_flag) const {
    switch (intersection_flag) {
        case PARTIALLY_CONTAINED: {
            return this->applyIntersectionTesting(intersecting);
        }
        case COMPLETELY_CONTAINED: {
            return bg::within(intersecting, this->getOuterPolygon());
        }
        default:
            return false;
    }
}

const std::vector<LaneletType> &Lanelet::getLaneletType() const {
    return laneletType;
}

void Lanelet::setLaneletType(const std::vector<LaneletType>& laType) {
    Lanelet::laneletType = laType;
}

const std::vector<ObstacleType> &Lanelet::getUserOneWay() const {
    return userOneWay;
}

void Lanelet::setUserOneWay(const std::vector<ObstacleType> &user) {
    Lanelet::userOneWay = user;
}

const std::vector<ObstacleType> &Lanelet::getUserBidirectional() const {
    return userBidirectional;
}

void Lanelet::setUserBidirectional(const std::vector<ObstacleType> &user) {
    Lanelet::userBidirectional = user;
}

Lanelet::Lanelet(size_t id, std::vector<vertice> centerVertices, std::vector<vertice> leftBorder,
                 std::vector<vertice> rightBorder,
                 std::vector<std::shared_ptr<Lanelet>> predecessorLanelets,
                 std::vector<std::shared_ptr<Lanelet>> successorLanelets,
                 std::vector<LaneletType> type, std::vector<ObstacleType> oneWay,
                 std::vector<ObstacleType> bidirectional) : id(id), centerVertices(std::move(centerVertices)),
                 leftBorder(std::move(leftBorder)), rightBorder(std::move(rightBorder)),
                 predecessorLanelets(std::move(predecessorLanelets)),
                 successorLanelets(std::move(successorLanelets)),
                               laneletType(std::move(type)), userOneWay(std::move(oneWay)),
                               userBidirectional(std::move(bidirectional)) {}

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

const Lanelet::adjacent &Lanelet::getAdjacentLeft() const {return adjacentLeft;}

void Lanelet::setAdjacentLeft(const Lanelet::adjacent &adjLeft) {adjacentLeft = adjLeft;}

const Lanelet::adjacent &Lanelet::getAdjacentRight() const {return adjacentRight;}

void Lanelet::setAdjacentRight(const Lanelet::adjacent &adjRight) {adjacentRight = adjRight;}
