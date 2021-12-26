//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "circle.h"

void Circle::setRadius(const double rad) { radius = rad; }

void Circle::setCenter(const double xPos, const double yPos) { center = vertex{xPos, yPos}; }

double Circle::getRadius() const { return radius; }

vertex Circle::getCenter() const { return center; }

void Circle::scaleShape(double factor) { this->setRadius(this->getRadius() * factor); }

ShapeType Circle::getType() { return ShapeType::circle; }