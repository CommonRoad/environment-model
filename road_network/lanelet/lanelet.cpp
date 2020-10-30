//
// Created by sebastian on 23.10.20.
//

#include "lanelet.h"

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
        vertice newVertice;
        newVertice.x = 0.5 * (leftBorder[i].x + rightBorder[i].x);
        newVertice.y = 0.5 * (leftBorder[i].y + rightBorder[i].y);
        addCenterVertice(newVertice);
    }
}

std::vector<vertice> Lanelet::getLeftBorderVerticesDirect() const { return leftBorder; }
std::vector<vertice> Lanelet::getRightBorderVerticesDirect() const { return rightBorder; }
std::vector<vertice> Lanelet::getCenterVerticesDirect() const { return centerVertices; }

const std::vector<vertice> &Lanelet::getCenterVertices() const { return centerVertices; }
const std::vector<vertice> &Lanelet::getLeftBorderVertices() const { return leftBorder; }
const std::vector<vertice> &Lanelet::getRightBorderVertices() const { return rightBorder; }

void Lanelet::addPredecessor(Lanelet *pre) { predecessorLanelets.push_back(pre); }
void Lanelet::addSuccessor(Lanelet *suc) { successorLanelets.push_back(suc); }

void Lanelet::setLeftAdjacent(Lanelet *left, std::string dir) {
    adjacentLeft.adj.push_back(left);
    adjacentLeft.dir = dir;
}

void Lanelet::setRightAdjacent(Lanelet *right, std::string dir) {
    adjacentRight.adj.push_back(right);
    adjacentRight.dir = dir;
}

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