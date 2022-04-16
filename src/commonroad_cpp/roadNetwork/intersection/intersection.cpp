//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "intersection.h"

#include <algorithm>
#include <deque>
#include <utility>

size_t Intersection::getId() const { return id; }

void Intersection::setId(size_t num) { Intersection::id = num; }

const std::vector<std::shared_ptr<Incoming>> &Intersection::getIncomings() const { return incomings; }

void Intersection::addIncoming(const std::shared_ptr<Incoming> &inc) {
    incomings.push_back(inc);
    memberLanelets.clear();
}

const std::vector<std::shared_ptr<Lanelet>> &Intersection::getCrossings() const { return crossings; }

void Intersection::setIncomings(const std::vector<std::shared_ptr<Incoming>> &incs) { incomings = incs; }

void Intersection::setCrossings(const std::vector<std::shared_ptr<Lanelet>> &cross) { crossings = cross; }

Intersection::Intersection(size_t intersectionId, std::vector<std::shared_ptr<Incoming>> incomings,
                           std::vector<std::shared_ptr<Lanelet>> crossings)
    : id(intersectionId), incomings(std::move(incomings)), crossings(std::move(crossings)) {}

const std::vector<std::shared_ptr<Lanelet>> &Intersection::getMemberLanelets() {
    if (memberLanelets.empty())
        computeMemberLanelets();
    return memberLanelets;
}

void Intersection::computeMemberLanelets() {
    memberLanelets = {};
    // collect outgoings
    for (const auto &incom : incomings) {
        for (const auto &let : incom->getIncomingLanelets())
            memberLanelets.push_back(let);
        for (const auto &let : incom->getLeftOutgoings()) {
            let->addLaneletType(LaneletType::intersectionLeftOutgoing);
            let->addLaneletType(LaneletType::intersectionLeftTurn);
            addIntersectionMemberLanelets(let, TurningDirections::left);
        }
        for (const auto &let : incom->getStraightOutgoings()) {
            let->addLaneletType(LaneletType::intersectionStraightOutgoing);
            let->addLaneletType(LaneletType::intersectionStraight);
            addIntersectionMemberLanelets(let, TurningDirections::straight);
        }
        for (const auto &let : incom->getRightOutgoings()) {
            let->addLaneletType(LaneletType::intersectionRightOutgoing);
            let->addLaneletType(LaneletType::intersectionRightTurn);
            addIntersectionMemberLanelets(let, TurningDirections::right);
        }
    }
}
void Intersection::addIntersectionMemberLanelets(const std::shared_ptr<Lanelet> &let, TurningDirections turn) {
    memberLanelets.push_back(let);
    std::deque<std::shared_ptr<Lanelet>> candidates{let->getPredecessors().begin(), let->getPredecessors().end()};
    while (!candidates.empty()) {
        std::shared_ptr<Lanelet> pre{candidates.front()};
        candidates.pop_front();
        if (!std::any_of(memberLanelets.begin(), memberLanelets.end(),
                         [pre](const std::shared_ptr<Lanelet> &tmp) { return tmp->getId() == pre->getId(); })) {
            memberLanelets.push_back(pre);
            auto pres{memberLanelets.back()->getPredecessors()};
            candidates.insert(candidates.end(), pres.begin(), pres.end());
            pre->addLaneletType(LaneletType::intersection);
            if (turn == TurningDirections::left)
                pre->addLaneletType(LaneletType::intersectionLeftTurn);
            else if (turn == TurningDirections::right)
                pre->addLaneletType(LaneletType::intersectionRightTurn);
            else if (turn == TurningDirections::straight)
                pre->addLaneletType(LaneletType::intersectionStraight);
        } else
            continue;
    }
}
