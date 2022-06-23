//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/roadNetwork/intersection/incoming.h>

#include <algorithm>
#include <deque>
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

std::vector<std::shared_ptr<Lanelet>> Incoming::getAllSuccessorLeft() {

    std::deque<std::shared_ptr<Lanelet>> candidates{leftOutgoings.begin(), leftOutgoings.end()};
    return collectIncomingSuccessors(candidates, false);
}

std::vector<std::shared_ptr<Lanelet>> Incoming::getAllSuccessorRight() {

    std::deque<std::shared_ptr<Lanelet>> candidates{rightOutgoings.begin(), rightOutgoings.end()};
    return collectIncomingSuccessors(candidates, false);
}

std::vector<std::shared_ptr<Lanelet>> Incoming::getAllSuccessorStraight() {

    std::deque<std::shared_ptr<Lanelet>> candidates{straightOutgoings.begin(), straightOutgoings.end()};
    return collectIncomingSuccessors(candidates, false);
}

std::vector<std::shared_ptr<Lanelet>> Incoming::getAllLeftTurningLanelets() {

    std::deque<std::shared_ptr<Lanelet>> candidates{leftOutgoings.begin(), leftOutgoings.end()};
    return collectIncomingSuccessors(candidates, true);
}

std::vector<std::shared_ptr<Lanelet>> Incoming::getAllRightTurningLanelets() {

    std::deque<std::shared_ptr<Lanelet>> candidates{rightOutgoings.begin(), rightOutgoings.end()};
    return collectIncomingSuccessors(candidates, true);
}

std::vector<std::shared_ptr<Lanelet>> Incoming::getAllStraightGoingLanelets() {

    std::deque<std::shared_ptr<Lanelet>> candidates{straightOutgoings.begin(), straightOutgoings.end()};
    return collectIncomingSuccessors(candidates, true);
}

std::vector<std::shared_ptr<Lanelet>>
Incoming::collectIncomingSuccessors(std::deque<std::shared_ptr<Lanelet>> &candidates, bool considerIncomings) {
    std::vector<std::shared_ptr<Lanelet>> memberLanelets;
    std::vector<std::shared_ptr<Lanelet>> incomings;
    while (!candidates.empty()) {
        std::shared_ptr<Lanelet> pre{candidates.front()};
        candidates.pop_front();
        if (!std::any_of(memberLanelets.begin(), memberLanelets.end(),
                         [pre](const std::shared_ptr<Lanelet> &tmp) { return tmp->getId() == pre->getId(); }) and
            (!std::any_of(incomingLanelets.begin(), incomingLanelets.end(),
                          [pre](const std::shared_ptr<Lanelet> &tmp) { return tmp->getId() == pre->getId(); }))) {
            memberLanelets.push_back(pre);
            auto pres{memberLanelets.back()->getPredecessors()};
            candidates.insert(candidates.end(), pres.begin(), pres.end());
        } else if (considerIncomings and
                   std::any_of(incomingLanelets.begin(), incomingLanelets.end(),
                               [pre](const std::shared_ptr<Lanelet> &tmp) { return tmp->getId() == pre->getId(); }))
            incomings.push_back(pre);
        else
            continue;
    }
    if (considerIncomings)
        memberLanelets.insert(memberLanelets.end(), incomings.begin(), incomings.end());
    return memberLanelets;
}

void Incoming::addIncomingLanelet(const std::shared_ptr<Lanelet> &incomingLanelet) {
    incomingLanelets.push_back(incomingLanelet);
}

void Incoming::addStraightOutgoing(const std::shared_ptr<Lanelet> &straightOutgoing) {
    straightOutgoings.push_back(straightOutgoing);
}

void Incoming::addLeftOutgoing(const std::shared_ptr<Lanelet> &leftOutgoing) { leftOutgoings.push_back(leftOutgoing); }

void Incoming::addRightOutgoing(const std::shared_ptr<Lanelet> &rightOutgoing) {
    rightOutgoings.push_back(rightOutgoing);
}

void Incoming::addOncoming(const std::shared_ptr<Lanelet> &oncoming) { oncomings.push_back(oncoming); }
