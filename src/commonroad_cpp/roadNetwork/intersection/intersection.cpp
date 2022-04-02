//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "intersection.h"

#include <algorithm>
#include <deque>
#include <unordered_set>
#include <utility>

size_t Intersection::getId() const { return id; }

void Intersection::setId(size_t num) { Intersection::id = num; }

const std::vector<std::shared_ptr<Incoming>> &Intersection::getIncomings() const { return incomings; }

void Intersection::addIncoming(const std::shared_ptr<Incoming> &inc) {
    incomings.push_back(inc);
    computeMemberLanelets();
}

const std::vector<std::shared_ptr<Lanelet>> &Intersection::getCrossings() const { return crossings; }

void Intersection::setIncomings(const std::vector<std::shared_ptr<Incoming>> &incs) {
    incomings = incs;
    computeMemberLanelets();
}

void Intersection::setCrossings(const std::vector<std::shared_ptr<Lanelet>> &cross) { crossings = cross; }

Intersection::Intersection(size_t intersectionId, std::vector<std::shared_ptr<Incoming>> incomings,
                           std::vector<std::shared_ptr<Lanelet>> crossings)
    : id(intersectionId), incomings(std::move(incomings)), crossings(std::move(crossings)) {
    computeMemberLanelets();
}

const std::vector<std::shared_ptr<Lanelet>> &Intersection::getMemberLanelets() const { return memberLanelets; }

void Intersection::computeMemberLanelets() {
    memberLanelets = {};
    std::unordered_set<size_t> outgoingIds;
    for (const auto &incom : incomings) {
        for (const auto &let : incom->getLeftOutgoings()) {
            memberLanelets.push_back(let);
            outgoingIds.insert(let->getId());
        }
        for (const auto &let : incom->getStraightOutgoings()) {
            memberLanelets.push_back(let);
            outgoingIds.insert(let->getId());
        }
        for (const auto &let : incom->getRightOutgoings()) {
            memberLanelets.push_back(let);
            outgoingIds.insert(let->getId());
        }
    }
    for (const auto &incom : incomings) {
        for (const auto &let : incom->getIncomingLanelets()) {
            memberLanelets.push_back(let);
            std::deque<std::shared_ptr<Lanelet>> candidates{let->getSuccessors().begin(), let->getSuccessors().end()};
            while (!candidates.empty()) {
                std::shared_ptr<Lanelet> suc{candidates.front()};
                candidates.pop_front();
                if (outgoingIds.find(suc->getId()) == outgoingIds.end() and
                    !std::any_of(memberLanelets.begin(), memberLanelets.end(),
                                 [suc](const std::shared_ptr<Lanelet> &let) { return let->getId() == suc->getId(); })) {
                    memberLanelets.push_back(suc);
                    auto succs{memberLanelets.back()->getSuccessors()};
                    candidates.insert(candidates.end(), succs.begin(), succs.end());
                } else
                    continue;
            }
        }
    }
}
