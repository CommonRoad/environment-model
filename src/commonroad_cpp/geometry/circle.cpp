//
// Created by Sebastian Maierhofer on 08.11.20.
//

#include "circle.h"

void Circle::setRadius(const double rad) { radius = rad; }

void Circle::setCenter(const double x, const double y) { center = vertex{x, y}; }

double Circle::getRadius() const { return radius; }

vertex Circle::getCenter() const { return center; }

void Circle::scaleShape(double factor) { this->setRadius(this->getRadius() * factor); }

ShapeType Circle::getType() { return ShapeType::circle; }

void Circle::printParameters() {
    std::cout << "--- Circle Shape ---" << std::endl;
    std::cout << "radius: " << this->getRadius() << std::endl;
    std::cout << "center: " << this->getCenter().x << this->getCenter().y << std::endl;
}
