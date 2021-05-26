//
// Created by Sebastian Maierhofer on 08.11.20.
//

#include "road_network.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <utility>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

RoadNetwork::RoadNetwork(const std::vector<std::shared_ptr<Lanelet>> &network,
                         std::vector<std::shared_ptr<Intersection>> inters,
                         std::vector<std::shared_ptr<TrafficSign>> signs,
                         std::vector<std::shared_ptr<TrafficLight>> lights)
    : laneletNetwork(network), intersections(std::move(inters)), trafficSigns(std::move(signs)),
      trafficLights(std::move(lights)) {
    // construct Rtree out of lanelets
    for (const std::shared_ptr<Lanelet> &la : network)
        rtree.insert(std::make_pair(la->getBoundingBox(), la->getId()));

    createLanes(network);
    setDynamicIntersectionLabels();
}

const std::vector<std::shared_ptr<Lanelet>> &RoadNetwork::getLaneletNetwork() const { return laneletNetwork; }

std::vector<std::shared_ptr<Lane>> RoadNetwork::getLanes() { return lanes; }

const std::vector<std::shared_ptr<Intersection>> &RoadNetwork::getIntersections() const { return intersections; }

void RoadNetwork::createLanes(const std::vector<std::shared_ptr<Lanelet>> &network) {
    std::vector<std::shared_ptr<Lanelet>> startLanelets;
    LaneletType laneletType;

    for (const auto &la : network) {
        if (la->getPredecessors().empty()) // if no predecessor -> use as start lanelet
            startLanelets.push_back(la);
        else {
            std::vector<std::shared_ptr<Lanelet>> predecessors;
            for (const std::shared_ptr<Lanelet> &pre : la->getPredecessors()) {
                predecessors.push_back(pre);
            }
            laneletType = extractClassifyingLaneletType(la);

            // if no predecessor with same classifying type exist -> use this lanelet as start lanelet
            for (const auto &pred : la->getPredecessors()) {
                if (!std::any_of(pred->getLaneletType().begin(), pred->getLaneletType().end(),
                                 [laneletType](LaneletType t) { return t == laneletType; })) {
                    startLanelets.push_back(la);
                    break;
                }
                break; // there exists at least one predecessor with same classifying type (e.g., at a merge)
            }
        }
    }
    // create lanes
    for (const auto &la : startLanelets) {
        //  laneletType = extractClassifyingLaneletType(la);
        auto newLanes{combineLaneletAndSuccessorsWithSameTypeToLane(la)};
        lanes.insert(lanes.end(), newLanes.begin(), newLanes.end());
    }
}

LaneletType RoadNetwork::extractClassifyingLaneletType(const std::shared_ptr<Lanelet> &la) {
    for (const auto &type : la->getLaneletType()) {
        if (type == LaneletType::accessRamp)
            return LaneletType::accessRamp;
        else if (type == LaneletType::exitRamp)
            return LaneletType::exitRamp;
        else if (type == LaneletType::mainCarriageWay)
            return LaneletType::mainCarriageWay;
        else if (type == LaneletType::shoulder)
            return LaneletType::shoulder;
        else if (type == LaneletType::urban)
            return LaneletType::urban;
    }
    return LaneletType::urban;
}

std::vector<std::shared_ptr<Lanelet>> RoadNetwork::findOccupiedLaneletsByShape(const polygon_type &polygonShape) {
    // find all relevant lanelets by making use of the rtree
    std::vector<value> relevantLanelets;
    rtree.query(bgi::intersects(bg::return_envelope<box>(polygonShape.outer())), std::back_inserter(relevantLanelets));
    std::vector<std::shared_ptr<Lanelet>> lanelets;
    for (auto la : relevantLanelets)
        lanelets.push_back(findLaneletById(static_cast<size_t>(la.second)));

    // check intersection with relevant lanelets
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets;
#pragma omp parallel for schedule(guided) shared(lanelets, occupiedLanelets, polygonShape) default(none)
    for (unsigned long i = 0; i < lanelets.size(); ++i) {
        std::shared_ptr<Lanelet> la{lanelets.at(i)};
        if (la->checkIntersection(polygonShape, ContainmentType::PARTIALLY_CONTAINED)) {
#pragma omp critical
            occupiedLanelets.push_back(la);
        }
    }
    return occupiedLanelets;
}

std::shared_ptr<Lane> RoadNetwork::findLaneByShape(const std::vector<std::shared_ptr<Lane>> &possibleLanes,
                                                   const polygon_type &polygonShape) {
    for (auto &possibleLane : possibleLanes)
        if (possibleLane->checkIntersection(polygonShape, ContainmentType::PARTIALLY_CONTAINED))
            return possibleLane;
    throw std::domain_error("shape does not occupy any of the provided lanes");
}

std::vector<std::shared_ptr<Lanelet>> RoadNetwork::findLaneletsByPosition(double xPos, double yPos) {
    std::vector<Lanelet> lanelet;
    polygon_type polygonPos;
    bg::append(polygonPos, point_type{xPos, yPos});

    return RoadNetwork::findOccupiedLaneletsByShape(polygonPos);
}

std::shared_ptr<Lanelet> RoadNetwork::findLaneletById(size_t id) {
    auto it = std::find_if(std::begin(laneletNetwork), std::end(laneletNetwork),
                           [id](auto val) { return val->getId() == id; });
    if (it == std::end(laneletNetwork))
        throw std::domain_error(std::to_string(id));

    return *it;
}

void RoadNetwork::setDynamicIntersectionLabels() {
    auto intersectionLaneletType = LaneletType::intersection;
    for (const auto &inters : intersections) {
        for (const auto &incom : inters->getIncomings()) {
            incom->setLeftOutgoings(extractOutgoingsFromIncoming(intersectionLaneletType, incom->getSuccessorsLeft()));
            incom->setRightOutgoings(
                extractOutgoingsFromIncoming(intersectionLaneletType, incom->getSuccessorsRight()));
            incom->setStraightOutgoings(
                extractOutgoingsFromIncoming(intersectionLaneletType, incom->getSuccessorsStraight()));
        }
    }
}

std::vector<std::shared_ptr<Lanelet>>
RoadNetwork::extractOutgoingsFromIncoming(const LaneletType &intersectionLaneletType,
                                          const std::vector<std::shared_ptr<Lanelet>> &incomingSuccessors) {
    std::vector<std::shared_ptr<Lanelet>> outgoings;
    for (const auto &inSuc : incomingSuccessors) {
        auto suc = inSuc;
        while (!std::all_of(
            suc->getSuccessors().begin(), suc->getSuccessors().end(),
            [intersectionLaneletType](auto laSuc) { return laSuc->hasLaneletType(intersectionLaneletType); }))
            suc = suc->getSuccessors().at(0); // we assume only one successor
        outgoings.push_back(suc);
    }
    return outgoings;
}

std::shared_ptr<Incoming> RoadNetwork::incomingOfLanelet(const std::shared_ptr<Lanelet> &lanelet) {
    for (const auto &inter : intersections)
        for (const auto &inco : inter->getIncomings())
            if (std::any_of(inco->getIncomingLanelets().begin(), inco->getIncomingLanelets().end(),
                            [lanelet](const std::shared_ptr<Lanelet> &la) { return la->getId() == lanelet->getId(); }))
                return inco;
    return nullptr;
}