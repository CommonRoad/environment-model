//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "incoming.h"

#include <utility>

size_t Incoming::getId() const { return id; }

void Incoming::setId(size_t index) { id = index; }

const std::vector<std::shared_ptr<Lanelet>> &Incoming::getIncomingLanelets() const { return incomingLanelets; }

void Incoming::setIncomingLanelets(const std::vector<std::shared_ptr<Lanelet>> &incLa) {
    // each incoming lanelet should be of type incoming
    for (const auto &let : incLa)
        let->addLaneletType(LaneletType::incoming);
    incomingLanelets = incLa;
}

const std::shared_ptr<Incoming> &Incoming::getIsLeftOf() const { return isLeftOf; }

void Incoming::setIsLeftOf(const std::shared_ptr<Incoming> &leftOf) { isLeftOf = leftOf; }

const std::vector<std::shared_ptr<Lanelet>> &Incoming::getStraightOutgoings() const { return straightOutgoings; }

void Incoming::setStraightOutgoings(const std::vector<std::shared_ptr<Lanelet>> &straight) {
    straightOutgoings = straight;
}

const std::vector<std::shared_ptr<Lanelet>> &Incoming::getLeftOutgoings() const { return leftOutgoings; }

void Incoming::setLeftOutgoings(const std::vector<std::shared_ptr<Lanelet>> &left) { leftOutgoings = left; }

const std::vector<std::shared_ptr<Lanelet>> &Incoming::getRightOutgoings() const { return rightOutgoings; }

void Incoming::setRightOutgoings(const std::vector<std::shared_ptr<Lanelet>> &right) { rightOutgoings = right; }

const std::vector<std::shared_ptr<Lanelet>> &Incoming::getOncomings() const { return oncomings; }

void Incoming::setOncomings(const std::vector<std::shared_ptr<Lanelet>> &oncomingLanelets) {
    oncomings = oncomingLanelets;
}
Incoming::Incoming(size_t incomingId, std::vector<std::shared_ptr<Lanelet>> incomingLanelets,
                   std::shared_ptr<Incoming> isLeftOf, std::vector<std::shared_ptr<Lanelet>> straightOutgoings,
                   std::vector<std::shared_ptr<Lanelet>> leftOutgoings,
                   std::vector<std::shared_ptr<Lanelet>> rightOutgoings,
                   std::vector<std::shared_ptr<Lanelet>> oncomings)
    : id(incomingId), incomingLanelets(std::move(incomingLanelets)), isLeftOf(std::move(isLeftOf)),
      straightOutgoings(std::move(straightOutgoings)), leftOutgoings(std::move(leftOutgoings)),
      rightOutgoings(std::move(rightOutgoings)), oncomings(std::move(oncomings)) {}
