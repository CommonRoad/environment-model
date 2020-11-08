//
// Created by sebastian on 08.11.20.
//

#include "rectangle.h"

void Rectangle::setLength(const double &l) { length = l; }

void Rectangle::setWidth(const double &w) { width = w; }

void Rectangle::setLength_raw(const double &raw_l) { raw_length = raw_l; }

void Rectangle::setWidth_raw(const double &raw_w) { raw_width = raw_w; }

double Rectangle::getLength() const { return length; }

double Rectangle::getWidth() const { return width; }

double Rectangle::getRawLength() const { return raw_length; }

double Rectangle::getRawWidth() const { return raw_width; }

void Rectangle::scaleShape(double factor) {
    this->setWidth(this->getWidth() * factor);
    this->setLength(this->getLength() * factor);
}

void Rectangle::printParameters() {
    std::cout << "--- Rectangle Shape ---" << std::endl;
    std::cout << "length with errors: " << this->getLength() << std::endl;
    std::cout << "width with errors: " << this->getWidth() << std::endl;
    std::cout << "length: " << this->getRawLength() << std::endl;
    std::cout << "width: " << this->getRawWidth() << std::endl;
}

std::string Rectangle::getType() { return "Rectangle"; }

