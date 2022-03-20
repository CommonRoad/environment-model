//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "intersection.h"

#include <utility>

size_t Intersection::getId() const { return id; }

void Intersection::setId(size_t num) { Intersection::id = num; }

const std::vector<std::shared_ptr<Incoming>> &Intersection::getIncomings() const { return incomings; }

void Intersection::addIncoming(const std::shared_ptr<Incoming> &inc) { incomings.push_back(inc); }

const std::vector<std::shared_ptr<Lanelet>> &Intersection::getCrossings() const { return crossings; }

void Intersection::setIncomings(const std::vector<std::shared_ptr<Incoming>> &incs) { incomings = incs; }

void Intersection::setCrossings(const std::vector<std::shared_ptr<Lanelet>> &cross) { crossings = cross; }

Intersection::Intersection(size_t intersectionId, std::vector<std::shared_ptr<Incoming>> incomings,
                           std::vector<std::shared_ptr<Lanelet>> crossings)
    : id(intersectionId), incomings(std::move(incomings)), crossings(std::move(crossings)) {}
