/*
 * circle.cpp
 *
 *  Created on: Jun 26, 2018
 *      Author: sebastian
 */

#include "circle.h"

/* setter functions */

void circle::setRadius(const double rad) { radius = rad; }

void circle::setCenter(const double x, const double y) { center = vertice{.x = x, .y = y}; }

/* getter functions */

double circle::getRadius() const { return radius; }

vertice circle::getCenter() const { return center; }

void circle::scaleShape(double factor) { this->setRadius(this->getRadius() * factor); }

std::string circle::getType() { return "Circle"; }