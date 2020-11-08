//
// Created by Sebastian Maierhofer on 01.11.20.
//

#include "incoming.h"

int Incoming::getId() const {
    return id;
}

void Incoming::setId(int id) {
    Incoming::id = id;
}

const std::vector<std::shared_ptr<Lanelet>> &Incoming::getIncomingLanelet() const {
    return incomingLanelet;
}

void Incoming::setIncomingLanelet(const std::vector<std::shared_ptr<Lanelet>> &incomingLanelet) {
    Incoming::incomingLanelet = incomingLanelet;
}

const std::vector<std::shared_ptr<Lanelet>> &Incoming::getSuccessorsRight() const {
    return successorsRight;
}

void Incoming::setSuccessorsRight(const std::vector<std::shared_ptr<Lanelet>> &successorsRight) {
    Incoming::successorsRight = successorsRight;
}

const std::vector<std::shared_ptr<Lanelet>> &Incoming::getSuccessorsStraight() const {
    return successorsStraight;
}

void Incoming::setSuccessorsStraight(const std::vector<std::shared_ptr<Lanelet>> &successorsStraight) {
    Incoming::successorsStraight = successorsStraight;
}

const std::vector<std::shared_ptr<Lanelet>> &Incoming::getSuccessorsLeft() const {
    return successorsLeft;
}

void Incoming::setSuccessorsLeft(const std::vector<std::shared_ptr<Lanelet>> &successorsLeft) {
    Incoming::successorsLeft = successorsLeft;
}

const std::shared_ptr<Incoming> &Incoming::getIsLeftOf() const {
    return isLeftOf;
}

void Incoming::setIsLeftOf(const std::shared_ptr<Incoming> &isLeftOf) {
    Incoming::isLeftOf = isLeftOf;
}
