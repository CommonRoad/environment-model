//
// Created by sebastian on 23.10.20.
//

#include "lanelet.h"

void lanelet::setLeftBorderVertices(const std::vector<vertice> &leftBorderVertices) { leftBorder = leftBorderVertices; }

void lanelet::setRightBorderVertices(const std::vector<vertice> &rightBorderVertices) {
    rightBorder = rightBorderVertices;
}

void lanelet::setCenterVertices(const std::vector<vertice> &center) { centerVertices = center; }

// Newly added functions which potentiall make setLeftBorder,... obsolete
// Move the information instead of a copy
void lanelet::moveLeftBorder(std::vector<vertice> &&leftBorderVertices) { leftBorder = std::move(leftBorderVertices); }

void lanelet::moveRightBorder(std::vector<vertice> &&rightBorderVertices) {
    rightBorder = std::move(rightBorderVertices);
}

void lanelet::moveCenterVertices(std::vector<vertice> &&center) { centerVertices = std::move(center); }

void lanelet::addLeftVertice(const vertice left) { leftBorder.push_back(left); }

void lanelet::addRightVertice(const vertice right) { rightBorder.push_back(right); }

void lanelet::addCenterVertice(const vertice center) { centerVertices.push_back(center); }

void lanelet::createCenterVertices() {
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

std::vector<vertice> lanelet::getLeftBorderVerticesDirect() const { return leftBorder; }
std::vector<vertice> lanelet::getRightBorderVerticesDirect() const { return rightBorder; }
std::vector<vertice> lanelet::getCenterVerticesDirect() const { return centerVertices; }

const std::vector<vertice> &lanelet::getCenterVertices() const { return centerVertices; }
const std::vector<vertice> &lanelet::getLeftBorderVertices() const { return leftBorder; }
const std::vector<vertice> &lanelet::getRightBorderVertices() const { return rightBorder; }
