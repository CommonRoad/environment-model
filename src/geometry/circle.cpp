//
// Created by Sebastian Maierhofer on 08.11.20.
//

#include "circle.h"

void Circle::setRadius(const double rad) { radius = rad; }

void Circle::setCenter(const double x, const double y) { center = vertice{.x = x, .y = y}; }

double Circle::getRadius() const { return radius; }

vertice Circle::getCenter() const { return center; }

void Circle::scaleShape(double factor) { this->setRadius(this->getRadius() * factor); }

ShapeType Circle::getType() { return ShapeType::circle; }