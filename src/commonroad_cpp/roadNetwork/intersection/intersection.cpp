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

#include <commonroad_cpp/roadNetwork/intersection/intersection_operations.h>

Intersection::Intersection(size_t intersectionId, std::vector<std::shared_ptr<IncomingGroup>> incomingGroups,
                           std::vector<std::shared_ptr<OutgoingGroup>> outgoingGroups)
    : id(intersectionId), incomings(std::move(incomingGroups)), outgoings(std::move(outgoingGroups)) {}

size_t Intersection::getId() const { return id; }

void Intersection::setId(size_t num) { Intersection::id = num; }

const std::vector<std::shared_ptr<IncomingGroup>> &Intersection::getIncomingGroups() const { return incomings; }

void Intersection::addIncomingGroup(const std::shared_ptr<IncomingGroup> &incoming) {
    incomings.push_back(incoming);
    memberLanelets.clear();
}

void Intersection::addOutgoingGroup(const std::shared_ptr<OutgoingGroup> &outgoing) {
    outgoings.push_back(outgoing);
    memberLanelets.clear();
}

const std::vector<std::shared_ptr<OutgoingGroup>> &Intersection::getOutgoingGroups() const { return outgoings; }

void Intersection::setIncomingGroups(const std::vector<std::shared_ptr<IncomingGroup>> &incs) { incomings = incs; }

void Intersection::setOutgoingGroups(const std::vector<std::shared_ptr<OutgoingGroup>> &outs) { outgoings = outs; }

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
            letInc->addLaneletType(LaneletType::incoming);
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
        intersection_operations::findLeftOf(incom, roadNetwork);
    }
}
