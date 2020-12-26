//
// Created by sebastian on 08.11.20.
//

#include "rectangle.h"

void Rectangle::setLength(const double len) { length = len; }

void Rectangle::setWidth(const double wid) { width = wid; }

double Rectangle::getLength() const { return length; }

double Rectangle::getWidth() const { return width; }

void Rectangle::scaleShape(double factor) {
    this->setWidth(this->getWidth() * factor);
    this->setLength(this->getLength() * factor);
}

void Rectangle::printParameters() {
    std::cout << "--- Rectangle Shape ---" << std::endl;
    std::cout << "length with errors: " << this->getLength() << std::endl;
    std::cout << "width with errors: " << this->getWidth() << std::endl;
}

std::string Rectangle::getType() { return "Rectangle"; }

