//
// Created by sebastian on 01.11.20.
//

#include "intersection.h"

int Intersection::getId() const {
    return id;
}

void Intersection::setId(int index) {
    Intersection::id = index;
}

const Incoming &Intersection::getIncoming() const {
    return incoming;
}

void Intersection::setIncoming(const Incoming &inc) {
    Intersection::incoming = inc;
}

const std::vector<std::vector<std::shared_ptr<Lanelet>>> &Intersection::getCrossing() const {
    return crossing;
}

void Intersection::setCrossing(const std::vector<std::vector<std::shared_ptr<Lanelet>>> &cr) {
    Intersection::crossing = cr;
}
