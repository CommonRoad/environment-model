//
// Created by Sebastian Maierhofer on 01.11.20.
//

#include "intersection.h"

int Intersection::getId() const { return id; }

void Intersection::setId(int num) { Intersection::id = num; }

const std::vector<std::shared_ptr<Incoming>> &Intersection::getIncomings() const { return incoming; }

void Intersection::addIncoming(const std::shared_ptr<Incoming> &inc) { incoming.push_back(inc); }

const std::vector<std::vector<std::shared_ptr<Lanelet>>> &Intersection::getCrossing() const { return crossing; }

void Intersection::setCrossing(const std::vector<std::vector<std::shared_ptr<Lanelet>>> &cr) {
    Intersection::crossing = cr;
}

