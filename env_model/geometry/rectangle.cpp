#include "rectangle.h"
#include <iostream>

void rectangle::setLength(const double &l) { length = l; }

void rectangle::setWidth(const double &w) { width = w; }

void rectangle::setLength_raw(const double &raw_l) { raw_length = raw_l; }

void rectangle::setWidth_raw(const double &raw_w) { raw_width = raw_w; }

double rectangle::getLength() const { return length; }

double rectangle::getWidth() const { return width; }

double rectangle::getRawLength() const { return raw_length; }

double rectangle::getRawWidth() const { return raw_width; }

void rectangle::scaleShape(double factor) {
    this->setWidth(this->getWidth() * factor);
    this->setLength(this->getLength() * factor);
}

void rectangle::printParameters() {
    std::cout << "--- rectangle shape ---" << std::endl;
    std::cout << "length with errors: " << this->getLength() << std::endl;
    std::cout << "width with errors: " << this->getWidth() << std::endl;
    std::cout << "length: " << this->getRawLength() << std::endl;
    std::cout << "width: " << this->getRawWidth() << std::endl;
}

std::string rectangle::getType() { return "Rectangle"; }
