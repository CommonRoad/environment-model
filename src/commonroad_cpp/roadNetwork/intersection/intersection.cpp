//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/roadNetwork/intersection/intersection.h>

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

void Intersection::addCrossing(const std::shared_ptr<Lanelet> &cross) { crossings.push_back(cross); }

Intersection::Intersection(size_t intersectionId, std::vector<std::shared_ptr<Incoming>> incomings,
                           std::vector<std::shared_ptr<Lanelet>> crossings)
    : id(intersectionId), incomings(std::move(incomings)), crossings(std::move(crossings)) {}

const std::vector<std::shared_ptr<Lanelet>> &
Intersection::getMemberLanelets(const std::shared_ptr<RoadNetwork> &roadNetwork) {
    if (memberLanelets.empty())
        computeMemberLanelets(roadNetwork);
    return memberLanelets;
}

void Intersection::computeMemberLanelets(const std::shared_ptr<RoadNetwork> &roadNetwork) {
    memberLanelets = {};
    // collect outgoings
    for (const auto &incom : incomings) {
        for (const auto &letInc : incom->getIncomingLanelets()) {
            memberLanelets.push_back(letInc);
            for (const auto &letOut : incom->getLeftOutgoings()) {
                letOut->addLaneletType(LaneletType::intersectionLeftOutgoing);
                letOut->addLaneletType(LaneletType::intersection);
                auto path{roadNetwork->getTopologicalMap()->findPaths(letInc->getId(), letOut->getId(), false)};
                for (const auto &pathLet : path) {
                    auto let{roadNetwork->findLaneletById(pathLet)};
                    if (!std::any_of(
                            memberLanelets.begin(), memberLanelets.end(),
                            [let](const std::shared_ptr<Lanelet> &tmp) { return tmp->getId() == let->getId(); })) {
                        memberLanelets.push_back(let);
                        let->addLaneletType(LaneletType::intersection);
                        let->addLaneletType(LaneletType::left);
                    }
                }
            }
            for (const auto &letOut : incom->getStraightOutgoings()) {
                letOut->addLaneletType(LaneletType::intersectionStraightOutgoing);
                letOut->addLaneletType(LaneletType::intersection);
                auto path{roadNetwork->getTopologicalMap()->findPaths(letInc->getId(), letOut->getId(), false)};
                for (const auto &pathLet : path) {
                    auto let{roadNetwork->findLaneletById(pathLet)};
                    if (!std::any_of(
                            memberLanelets.begin(), memberLanelets.end(),
                            [let](const std::shared_ptr<Lanelet> &tmp) { return tmp->getId() == let->getId(); })) {
                        memberLanelets.push_back(let);
                        let->addLaneletType(LaneletType::intersection);
                        let->addLaneletType(LaneletType::straight);
                    }
                }
            }
            for (const auto &letOut : incom->getRightOutgoings()) {
                letOut->addLaneletType(LaneletType::intersectionRightOutgoing);
                letOut->addLaneletType(LaneletType::intersection);
                auto path{roadNetwork->getTopologicalMap()->findPaths(letInc->getId(), letOut->getId(), false)};
                for (const auto &pathLet : path) {
                    auto let{roadNetwork->findLaneletById(pathLet)};
                    if (!std::any_of(
                            memberLanelets.begin(), memberLanelets.end(),
                            [let](const std::shared_ptr<Lanelet> &tmp) { return tmp->getId() == let->getId(); })) {
                        memberLanelets.push_back(let);
                        let->addLaneletType(LaneletType::intersection);
                        let->addLaneletType(LaneletType::right);
                    }
                }
            }
        }
    }
}
