//
// Created by Sebastian Maierhofer on 23.10.20.
//
#include "lanelet.h"
#include <utility>

Lanelet::Lanelet(size_t id, std::vector<vertex> leftBorder, std::vector<vertex> rightBorder,
                 std::vector<LaneletType> type, std::vector<ObstacleType> oneWay,
                 std::vector<ObstacleType> userBidirectional)
    : id{id}, leftBorder{std::move(leftBorder)}, rightBorder{std::move(rightBorder)}, laneletType{std::move(type)},
      userOneWay{std::move(oneWay)}, userBidirectional{std::move(userBidirectional)} {
    createCenterVertices();
    constructOuterPolygon();
}

Lanelet::Lanelet(size_t id, std::vector<vertex> leftBorder, std::vector<vertex> rightBorder,
                 std::vector<std::shared_ptr<Lanelet>> predecessorLanelets,
                 std::vector<std::shared_ptr<Lanelet>> successorLanelets, std::vector<LaneletType> type,
                 std::vector<ObstacleType> oneWay = std::vector<ObstacleType>(),
                 std::vector<ObstacleType> userBidirectional = std::vector<ObstacleType>())
    : id{id}, leftBorder{std::move(leftBorder)}, rightBorder{std::move(rightBorder)},
      predecessorLanelets{std::move(predecessorLanelets)}, successorLanelets{std::move(successorLanelets)},
      laneletType{std::move(type)}, userOneWay{std::move(oneWay)}, userBidirectional{std::move(userBidirectional)} {
    createCenterVertices();
    constructOuterPolygon();
}

void Lanelet::setId(const size_t laneletId) { id = laneletId; }

void Lanelet::setLeftAdjacent(const std::shared_ptr<Lanelet> &left, DrivingDirection dir) {
    adjacentLeft.adj = left;
    adjacentLeft.dir = dir;
}

void Lanelet::setRightAdjacent(const std::shared_ptr<Lanelet> &right, DrivingDirection dir) {
    adjacentRight.adj = right;
    adjacentRight.dir = dir;
}

void Lanelet::setLeftBorderVertices(const std::vector<vertex> &leftBorderVertices) { leftBorder = leftBorderVertices; }

void Lanelet::setRightBorderVertices(const std::vector<vertex> &rightBorderVertices) {
    rightBorder = rightBorderVertices;
}

void Lanelet::setLaneletType(const std::vector<LaneletType> &laType) { laneletType = laType; }

void Lanelet::setUserOneWay(const std::vector<ObstacleType> &user) { userOneWay = user; }

void Lanelet::setUserBidirectional(const std::vector<ObstacleType> &user) { userBidirectional = user; }

void Lanelet::setStopLine(const std::shared_ptr<StopLine> &sl) { stopLine = sl; }

void Lanelet::addLeftVertex(const vertex left) { leftBorder.push_back(left); }

void Lanelet::addRightVertex(const vertex right) { rightBorder.push_back(right); }

void Lanelet::addCenterVertex(const vertex center) { centerVertices.push_back(center); }

void Lanelet::addPredecessor(const std::shared_ptr<Lanelet> &pre) { predecessorLanelets.push_back(pre); }

void Lanelet::addSuccessor(const std::shared_ptr<Lanelet> &suc) { successorLanelets.push_back(suc); }

void Lanelet::addTrafficLight(const std::shared_ptr<TrafficLight> &light) { trafficLights.push_back(light); }

void Lanelet::addTrafficSign(const std::shared_ptr<TrafficSign> &sign) { trafficSigns.push_back(sign); }

size_t Lanelet::getId() const { return id; }

std::vector<std::shared_ptr<Lanelet>> Lanelet::getPredecessors() const { return predecessorLanelets; }

std::vector<std::shared_ptr<Lanelet>> Lanelet::getSuccessors() const { return successorLanelets; }

const std::vector<vertex> &Lanelet::getCenterVertices() const { return centerVertices; }

const std::vector<vertex> &Lanelet::getLeftBorderVertices() const { return leftBorder; }

const std::vector<vertex> &Lanelet::getRightBorderVertices() const { return rightBorder; }

std::vector<std::shared_ptr<TrafficLight>> Lanelet::getTrafficLights() const { return trafficLights; }

std::vector<std::shared_ptr<TrafficSign>> Lanelet::getTrafficSigns() const { return trafficSigns; }

const polygon_type &Lanelet::getOuterPolygon() const { return outerPolygon; }

const box &Lanelet::getBoundingBox() const { return boundingBox; }

const std::vector<LaneletType> &Lanelet::getLaneletType() const { return laneletType; }

const std::vector<ObstacleType> &Lanelet::getUserOneWay() const { return userOneWay; }

const std::vector<ObstacleType> &Lanelet::getUserBidirectional() const { return userBidirectional; }

const Lanelet::adjacent &Lanelet::getAdjacentLeft() const { return adjacentLeft; }

const Lanelet::adjacent &Lanelet::getAdjacentRight() const { return adjacentRight; }

const std::shared_ptr<StopLine> &Lanelet::getStopLine() const { return stopLine; }

bool Lanelet::applyIntersectionTesting(const polygon_type &polygon_shape) const {
    // check first if shape intersects with bounding box since this evaluation is faster
    return bg::intersects(polygon_shape, this->getBoundingBox()) &&
           bg::intersects(polygon_shape, this->getOuterPolygon());
}

bool Lanelet::checkIntersection(const polygon_type &polygon_shape, ContainmentType intersection_type) const {
    switch (intersection_type) {
    case ContainmentType::PARTIALLY_CONTAINED: {
        return this->applyIntersectionTesting(polygon_shape);
    }
    case ContainmentType::COMPLETELY_CONTAINED: {
        return bg::within(polygon_shape, this->getOuterPolygon());
    }
    default:
        return false;
    }
}

void Lanelet::constructOuterPolygon() {
    const std::vector<vertex> &leftBorderTemp = this->getLeftBorderVertices();
    const std::vector<vertex> &rightBorderTemp = this->getRightBorderVertices();

    if (!leftBorderTemp.empty()) {
        size_t idx = 0;
        polygon_type polygon;
        polygon.outer().resize(leftBorderTemp.size() + rightBorderTemp.size() + 1);

        for (const auto &it : leftBorderTemp) {
            polygon.outer()[idx] = point_type{it.x, it.y};
            idx++;
        }
        for (const auto &it : boost::adaptors::reverse(rightBorderTemp)) {
            polygon.outer()[idx] = point_type{it.x, it.y};
            idx++;
        }
        polygon.outer().back() = point_type{leftBorderTemp[0].x, leftBorderTemp[0].y};

        // Improve polygon (remove duplicated vertices, close vertices, order)
        bg::simplify(polygon, outerPolygon, 0.01);
        bg::unique(outerPolygon);
        bg::correct(outerPolygon);

        bg::envelope(outerPolygon, boundingBox); // set bounding box
    }
}

void Lanelet::createCenterVertices() {
    unsigned long numVertices = leftBorder.size();
    for (unsigned long i = 0; i < numVertices; i++) {
        vertex newVertex{};
        // calculate x and y values separately in order to minimize error
        newVertex.x = 0.5 * (leftBorder[i].x + rightBorder[i].x);
        newVertex.y = 0.5 * (leftBorder[i].y + rightBorder[i].y);
        addCenterVertex(newVertex);
    }
}

double Lanelet::getOrientationAtPosition(double positionX, double positionY) {
    // find closest vertex to the given position
    std::vector<double> dif(centerVertices.size() - 1);
    for (unsigned long i = 0; i < centerVertices.size() - 1; ++i) {
        vertex vert{centerVertices[i]};
        dif[i] = sqrt(pow(vert.x - positionX, 2) + pow(vert.y - positionY, 2));
    }
    unsigned long closestIndex{static_cast<unsigned long>(std::min_element(dif.begin(), dif.end()) - dif.begin())};

    // calculate orientation at vertex using its successor vertex
    vertex vert1{centerVertices[closestIndex]};
    vertex vert2{centerVertices[closestIndex + 1]};
    return atan2(vert2.y - vert1.y, vert2.x - vert1.x);
}

bool Lanelet::hasLaneletType(LaneletType laType) {
    return std::any_of(laneletType.begin(), laneletType.end(), [laType](auto ty) { return ty == laType; });
}

void Lanelet::addLaneletType(LaneletType laType) { laneletType.push_back(laType); }

LineMarking Lanelet::getLineMarkingLeft() const { return lineMarkingLeft; }

void Lanelet::setLineMarkingLeft(LineMarking marking) { lineMarkingLeft = marking; }

LineMarking Lanelet::getLineMarkingRight() const { return lineMarkingRight; }

void Lanelet::setLineMarkingRight(LineMarking marking) { lineMarkingRight = marking; }
