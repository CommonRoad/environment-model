//
// Created by Sebastian Maierhofer on 01.11.20.
//

#include "intersection.h"

int Intersection::getId() const { return id; }

void Intersection::setId(int num) { Intersection::id = num; }

const std::vector<std::shared_ptr<Incoming>> &Intersection::getIncomings() const { return incomings; }

void Intersection::addIncoming(const std::shared_ptr<Incoming> &inc) { incomings.push_back(inc); }

const std::vector<std::shared_ptr<Lanelet>> &Intersection::getCrossings() const { return crossings; }

void Intersection::setIncomings(const std::vector<std::shared_ptr<Incoming>> &incs) { incomings = incs; }

void Intersection::setCrossings(const std::vector<std::shared_ptr<Lanelet>> &cr) { crossings = cr; }
