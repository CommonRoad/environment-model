//
// Created by Sebastian Maierhofer on 01.11.20.
//

#include "incoming.h"

int Incoming::getId() const {return id;}

void Incoming::setId(int index) {id = index;}

const std::vector<std::shared_ptr<Lanelet>> &Incoming::getIncomingLanelet() const {return incomingLanelet;}

void Incoming::setIncomingLanelet(const std::vector<std::shared_ptr<Lanelet>> &incLa) {incomingLanelet = incLa;}

const std::vector<std::shared_ptr<Lanelet>> &Incoming::getSuccessorsRight() const {return successorsRight;}

void Incoming::setSuccessorsRight(const std::vector<std::shared_ptr<Lanelet>> &sucRight) {successorsRight = sucRight;}

const std::vector<std::shared_ptr<Lanelet>> &Incoming::getSuccessorsStraight() const {return successorsStraight;}

void Incoming::setSuccessorsStraight(const std::vector<std::shared_ptr<Lanelet>> &sucStraight) {successorsStraight = sucStraight;}

const std::vector<std::shared_ptr<Lanelet>> &Incoming::getSuccessorsLeft() const {return successorsLeft;}

void Incoming::setSuccessorsLeft(const std::vector<std::shared_ptr<Lanelet>> &sucLeft) {successorsLeft = sucLeft;}

const std::shared_ptr<Incoming> &Incoming::getIsLeftOf() const {return isLeftOf;}

void Incoming::setIsLeftOf(const std::shared_ptr<Incoming> &leftOf) {isLeftOf = leftOf;}

