//
// Created by Sebastian Maierhofer on 01.11.20.
//

#include "incoming.h"

int Incoming::getId() const { return id; }

void Incoming::setId(int index) { id = index; }

const std::vector<std::shared_ptr<Lanelet>> &Incoming::getIncomingLanelets() const { return incomingLanelets; }

void Incoming::setIncomingLanelets(const std::vector<std::shared_ptr<Lanelet>> &incLa) { incomingLanelets = incLa; }

const std::vector<std::shared_ptr<Lanelet>> &Incoming::getSuccessorsRight() const { return successorsRight; }

void Incoming::setSuccessorsRight(const std::vector<std::shared_ptr<Lanelet>> &sucRight) { successorsRight = sucRight; }

const std::vector<std::shared_ptr<Lanelet>> &Incoming::getSuccessorsStraight() const { return successorsStraight; }

void Incoming::setSuccessorsStraight(const std::vector<std::shared_ptr<Lanelet>> &sucStraight) {
    successorsStraight = sucStraight;
}

const std::vector<std::shared_ptr<Lanelet>> &Incoming::getSuccessorsLeft() const { return successorsLeft; }

void Incoming::setSuccessorsLeft(const std::vector<std::shared_ptr<Lanelet>> &sucLeft) { successorsLeft = sucLeft; }

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
