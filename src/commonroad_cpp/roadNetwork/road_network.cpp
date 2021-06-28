//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <utility>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/parameters.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <commonroad_cpp/auxiliaryDefs/traffic_signs.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>

#include "road_network.h"

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

struct RoadNetwork::impl {
    bgi::rtree<value, bgi::quadratic<16>>
        rtree; //**< rtree defined by lanelets of road network for faster occupancy calculation*/
};

RoadNetwork::RoadNetwork(RoadNetwork &&) = default;
RoadNetwork::~RoadNetwork() = default;
RoadNetwork &RoadNetwork::operator=(RoadNetwork &&) = default;

RoadNetwork::RoadNetwork(const std::vector<std::shared_ptr<Lanelet>> &network, SupportedTrafficSignCountry cou,
                         std::vector<std::shared_ptr<TrafficSign>> signs,
                         std::vector<std::shared_ptr<TrafficLight>> lights,
                         std::vector<std::shared_ptr<Intersection>> inters)
    : laneletNetwork(network), country(cou), trafficSigns(std::move(signs)), trafficLights(std::move(lights)),
      intersections(std::move(inters)), pImpl(std::make_unique<impl>()) {
    // construct Rtree out of lanelets
    for (const std::shared_ptr<Lanelet> &la : network)
        pImpl->rtree.insert(std::make_pair(la->getBoundingBox(), la->getId()));
    trafficSignIDLookupTable = TrafficSignLookupTableByCountry.at(cou);
    createLanes(network);
}

const std::vector<std::shared_ptr<Lanelet>> &RoadNetwork::getLaneletNetwork() const { return laneletNetwork; }

std::vector<std::shared_ptr<Lane>> RoadNetwork::getLanes() { return lanes; }

const std::vector<std::shared_ptr<Intersection>> &RoadNetwork::getIntersections() const { return intersections; }

void RoadNetwork::createLanes(const std::vector<std::shared_ptr<Lanelet>> &network) {
    std::vector<std::shared_ptr<Lanelet>> startLanelets;
    std::set<LaneletType> classifyinglaneletTypes{LaneletType::incoming, LaneletType::shoulder, LaneletType::accessRamp,
                                                  LaneletType::exitRamp};

    for (const auto &la : network) {
        if (la->getPredecessors().empty()) // if no predecessor -> use as start lanelet
            startLanelets.push_back(la);
        else {
            std::vector<std::shared_ptr<Lanelet>> predecessors;
            for (const std::shared_ptr<Lanelet> &pre : la->getPredecessors()) {
                predecessors.push_back(pre);
            }

            // if no predecessor with same classifying type exist -> use this lanelet as start lanelet
            std::set<LaneletType> intersectingLa;
            std::set_intersection(la->getLaneletType().begin(), la->getLaneletType().end(),
                                  classifyinglaneletTypes.begin(), classifyinglaneletTypes.end(),
                                  std::inserter(intersectingLa, intersectingLa.begin()));
            for (const auto &pred : la->getPredecessors()) {
                std::set<LaneletType> intersectingPred;
                std::set_intersection(pred->getLaneletType().begin(), pred->getLaneletType().end(),
                                      classifyinglaneletTypes.begin(), classifyinglaneletTypes.end(),
                                      std::inserter(intersectingPred, intersectingPred.begin()));
                if (intersectingPred.empty() != intersectingLa.empty()) {
                    startLanelets.push_back(la);
                    break;
                }
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

std::vector<std::shared_ptr<Lanelet>> RoadNetwork::findOccupiedLaneletsByShape(const polygon_type &polygonShape) {
    // find all relevant lanelets by making use of the rtree
    std::vector<value> relevantLanelets;
    pImpl->rtree.query(bgi::intersects(bg::return_envelope<box>(polygonShape.outer())),
                       std::back_inserter(relevantLanelets));
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
    for (const auto &possibleLane : possibleLanes)
        if (possibleLane->checkIntersection(polygonShape, ContainmentType::PARTIALLY_CONTAINED))
            return possibleLane;
    return {}; // TODO think about better solution
    // throw std::domain_error("shape does not occupy any of the provided lanes");
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

std::shared_ptr<Incoming> RoadNetwork::incomingOfLanelet(const std::shared_ptr<Lanelet> &lanelet) {
    for (const auto &inter : intersections)
        for (const auto &inco : inter->getIncomings())
            if (std::any_of(inco->getIncomingLanelets().begin(), inco->getIncomingLanelets().end(),
                            [lanelet](const std::shared_ptr<Lanelet> &la) { return la->getId() == lanelet->getId(); }))
                return inco;
    return nullptr;
}

SupportedTrafficSignCountry RoadNetwork::getCountry() const { return country; }

SupportedTrafficSignCountry RoadNetwork::matchStringToCountry(const std::string &name) {
    if (name == "DEU")
        return SupportedTrafficSignCountry::GERMANY;
    else if (name == "USA")
        return SupportedTrafficSignCountry::USA;
    else if (name == "ESP")
        return SupportedTrafficSignCountry::SPAIN;
    else
        return SupportedTrafficSignCountry::ZAMUNDA;
}
std::string RoadNetwork::extractTrafficSignIDForCountry(TrafficSignTypes type) {
    return trafficSignIDLookupTable->at(type);
}
