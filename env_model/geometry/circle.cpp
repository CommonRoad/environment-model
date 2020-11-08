//
// Created by sebastian on 08.11.20.
//

#include "circle.h"

/* setter functions */

void Circle::setRadius(const double rad) { radius = rad; }

void Circle::setCenter(const double x, const double y) { center = vertice{.x = x, .y = y}; }


/* getter functions */

double Circle::getRadius() const { return radius; }

vertice Circle::getCenter() const { return center; }

void Circle::scaleShape(double factor) { this->setRadius(this->getRadius() * factor); }

std::string Circle::getType() { return "Circle"; }