#include "commonroad_cpp/roadNetwork/intersection/crossing_group.h"
#include "commonroad_cpp/roadNetwork/intersection/incoming_group.h"
#include "commonroad_cpp/roadNetwork/intersection/outgoing_group.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include <algorithm>
#include <commonroad_cpp/geometry/geometric_operations.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection_operations.h>
#include <utility>

Intersection::Intersection(const size_t intersectionId, std::vector<std::shared_ptr<IncomingGroup>> incomingGroups,
                           std::vector<std::shared_ptr<OutgoingGroup>> outgoingGroups,
                           std::vector<std::shared_ptr<CrossingGroup>> crossingGroups)
    : id(intersectionId), incomings(std::move(incomingGroups)), outgoings(std::move(outgoingGroups)),
      crossings(std::move(crossingGroups)) {
    determineIntersectionType();
}

size_t Intersection::getId() const { return id; }

void Intersection::setId(const size_t num) { id = num; }

const std::vector<std::shared_ptr<IncomingGroup>> &Intersection::getIncomingGroups() const { return incomings; }

void Intersection::addIncomingGroup(const std::shared_ptr<IncomingGroup> &incoming) {
    incomings.push_back(incoming);
    memberLanelets.clear();
    memberLaneletIds_.clear();
    intersectionTypes.clear();
}

void Intersection::addOutgoingGroup(const std::shared_ptr<OutgoingGroup> &outgoing) {
    outgoings.push_back(outgoing);
    memberLanelets.clear();
    memberLaneletIds_.clear();
    intersectionTypes.clear();
}

const std::vector<std::shared_ptr<OutgoingGroup>> &Intersection::getOutgoingGroups() const { return outgoings; }

void Intersection::setIncomingGroups(const std::vector<std::shared_ptr<IncomingGroup>> &incs) { incomings = incs; }

void Intersection::setOutgoingGroups(const std::vector<std::shared_ptr<OutgoingGroup>> &outs) { outgoings = outs; }

const std::vector<std::shared_ptr<Lanelet>> &
Intersection::getMemberLanelets(const std::shared_ptr<RoadNetwork> &roadNetwork) {
    if (memberLanelets.empty())
        computeMemberLanelets(roadNetwork);
    if (memberLaneletIds_.empty() && !memberLanelets.empty()) {
        memberLaneletIds_.reserve(memberLanelets.size());
        for (const auto &la : memberLanelets)
            memberLaneletIds_.emplace(la->getId());
    }
    return memberLanelets;
}

void Intersection::computeMemberLanelets(const std::shared_ptr<RoadNetwork> &roadNetwork) {
    memberLanelets = {};
    memberLaneletIds_.clear();

    // Helper: add a lanelet to memberLanelets if not already present (O(1) via memberLaneletIds_).
    auto addMember = [this](const std::shared_ptr<Lanelet> &let) -> bool {
        if (memberLaneletIds_.insert(let->getId()).second) {
            memberLanelets.push_back(let);
            return true;
        }
        return false;
    };

    for (const auto &incom : incomings) {
        for (const auto &letInc : incom->getIncomingLanelets()) {
            letInc->addLaneletType(LaneletType::incoming);
            addMember(letInc);

            // Build set of outgoing endpoint IDs for early-exit during path traversal.
            std::unordered_set<size_t> allOutgoingIds;
            for (const auto &l : incom->getLeftOutgoings())
                allOutgoingIds.insert(l->getId());
            for (const auto &l : incom->getStraightOutgoings())
                allOutgoingIds.insert(l->getId());
            for (const auto &l : incom->getRightOutgoings())
                allOutgoingIds.insert(l->getId());

            for (const auto &letOut : incom->getLeftOutgoings()) {
                letOut->addLaneletType(LaneletType::intersectionLeftOutgoing);
                letOut->addLaneletType(LaneletType::intersection);
                letOut->addLaneletType(LaneletType::left);
                addMember(letOut);
                auto path{roadNetwork->getTopologicalMap()->findPaths(letInc->getId(), letOut->getId(), false)};
                for (const auto &pathLet : path) {
                    if (allOutgoingIds.count(pathLet))
                        break;
                    auto let{roadNetwork->findLaneletById(pathLet)};
                    if (addMember(let)) {
                        let->addLaneletType(LaneletType::intersection);
                        let->addLaneletType(LaneletType::left);
                    }
                }
            }
            for (const auto &letOut : incom->getStraightOutgoings()) {
                letOut->addLaneletType(LaneletType::intersectionStraightOutgoing);
                letOut->addLaneletType(LaneletType::straight);
                letOut->addLaneletType(LaneletType::intersection);
                addMember(letOut);
                auto path{roadNetwork->getTopologicalMap()->findPaths(letInc->getId(), letOut->getId(), false)};
                for (const auto &pathLet : path) {
                    if (allOutgoingIds.count(pathLet))
                        break;
                    auto let{roadNetwork->findLaneletById(pathLet)};
                    if (addMember(let)) {
                        let->addLaneletType(LaneletType::intersection);
                        let->addLaneletType(LaneletType::straight);
                    }
                }
            }
            for (const auto &letOut : incom->getRightOutgoings()) {
                letOut->addLaneletType(LaneletType::intersectionRightOutgoing);
                letOut->addLaneletType(LaneletType::right);
                letOut->addLaneletType(LaneletType::intersection);
                addMember(letOut);
                auto path{roadNetwork->getTopologicalMap()->findPaths(letInc->getId(), letOut->getId(), false)};
                for (const auto &pathLet : path) {
                    if (allOutgoingIds.count(pathLet))
                        break;
                    auto let{roadNetwork->findLaneletById(pathLet)};
                    if (addMember(let)) {
                        let->addLaneletType(LaneletType::intersection);
                        let->addLaneletType(LaneletType::right);
                    }
                }
            }
        }
        intersection_operations::findLeftOf(incom, roadNetwork);
    }
}
void Intersection::setCrossingGroups(const std::vector<std::shared_ptr<CrossingGroup>> &cros) { crossings = cros; }

void Intersection::addCrossingGroup(const std::shared_ptr<CrossingGroup> &crossing) { crossings.push_back(crossing); }

const std::vector<std::shared_ptr<CrossingGroup>> &Intersection::getCrossingGroups() const { return crossings; }

bool Intersection::hasIntersectionType(const IntersectionType type) {
    if (intersectionTypes.empty())
        determineIntersectionType();
    if (intersectionTypes.find(type) != intersectionTypes.end())
        return true;
    return false;
}

void Intersection::determineIntersectionType() {
    bool hasFourWayStop{false};
    if (incomings.size() == 4) {
        intersectionTypes.insert(IntersectionType::FOUR_WAY_INTERSECTION);
        // if lanelet from each incoming references a STOP sign, it is 4 way stop
        if (std::all_of(incomings.begin(), incomings.end(), [](const std::shared_ptr<IncomingGroup> &incoming) {
                return std::any_of(incoming->getIncomingLanelets().begin(), incoming->getIncomingLanelets().end(),
                                   [](const std::shared_ptr<Lanelet> &la) {
                                       return la->hasTrafficSign(TrafficSignTypes::STOP_4_WAY);
                                   });
            })) {
            intersectionTypes.insert(IntersectionType::FOUR_WAY_STOP_INTERSECTION);
            hasFourWayStop = true;
        }
    } else if (incomings.size() == 3) {
        bool hasTIntersection{false};
        const auto incoming_1 = incomings.at(0)->getIncomingLanelets().at(0);
        const auto incoming_2 = incomings.at(1)->getIncomingLanelets().at(0);
        const auto incoming_3 = incomings.at(2)->getIncomingLanelets().at(0);

        // get the orientation from each incoming lanelet with the vertices
        const auto orientation_incoming_1 = geometric_operations::getOrientationInDeg(incoming_1);
        const auto orientation_incoming_2 = geometric_operations::getOrientationInDeg(incoming_2);
        const auto orientation_incoming_3 = geometric_operations::getOrientationInDeg(incoming_3);

        // now get through the three cases how the incomings can be located
        // first case: T is incoming_1
        if (geometric_operations::is90Deg(orientation_incoming_1, orientation_incoming_2) &&
            geometric_operations::is90Deg(orientation_incoming_1, orientation_incoming_3)) {
            if (geometric_operations::is180Deg(orientation_incoming_2, orientation_incoming_3)) {
                intersectionTypes.insert(IntersectionType::T_INTERSECTION);
                hasTIntersection = true;
            }
        }
        // second case: T is incoming_2
        if (!hasTIntersection and geometric_operations::is90Deg(orientation_incoming_2, orientation_incoming_1) &&
            geometric_operations::is90Deg(orientation_incoming_2, orientation_incoming_3)) {
            if (geometric_operations::is180Deg(orientation_incoming_1, orientation_incoming_3)) {
                intersectionTypes.insert(IntersectionType::T_INTERSECTION);
                hasTIntersection = true;
            }
        }
        // third case: T is incoming_3
        if (!hasTIntersection and geometric_operations::is90Deg(orientation_incoming_3, orientation_incoming_1) &&
            geometric_operations::is90Deg(orientation_incoming_3, orientation_incoming_2)) {
            if (geometric_operations::is180Deg(orientation_incoming_1, orientation_incoming_2)) {
                intersectionTypes.insert(IntersectionType::T_INTERSECTION);
            }
        }
    }
    if (!hasFourWayStop) {
        if (std::all_of(incomings.begin(), incomings.end(), [](const std::shared_ptr<IncomingGroup> &incoming) {
                return std::all_of(incoming->getIncomingLanelets().begin(), incoming->getIncomingLanelets().end(),
                                   [](const std::shared_ptr<Lanelet> &la) {
                                       return la->getTrafficLights().empty() and la->getTrafficSigns().empty() and
                                              la->getStopLine() == nullptr;
                                   });
            })) {
            intersectionTypes.insert(IntersectionType::UNCONTROLLED_INTERSECTION);
        }
    }
    if (intersectionTypes.empty())
        intersectionTypes.insert(IntersectionType::UNKNOWN);
}

bool Intersection::isMemberLanelet(const size_t memberLanelet) {
    if (memberLaneletIds_.empty() && !memberLanelets.empty()) {
        for (const auto &la : memberLanelets)
            memberLaneletIds_.emplace(la->getId());
    }
    return memberLaneletIds_.count(memberLanelet) > 0;
}
