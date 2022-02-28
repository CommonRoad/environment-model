//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <utility>

#include <boost/geometry/index/parameters.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/strategies/cartesian.hpp>

#include <commonroad_cpp/auxiliaryDefs/traffic_signs.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>

#include "road_network.h"

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

struct RoadNetwork::impl {
    bgi::rtree<value, bgi::quadratic<16>>
        rtree; //**< rtree defined by lanelets of road network for faster occupancy calculation*/
};

RoadNetwork::RoadNetwork(RoadNetwork &&) noexcept = default;

RoadNetwork::~RoadNetwork() = default;

RoadNetwork &RoadNetwork::operator=(RoadNetwork &&) noexcept = default;

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
}

const std::vector<std::shared_ptr<Lanelet>> &RoadNetwork::getLaneletNetwork() const { return laneletNetwork; }

std::vector<std::shared_ptr<Lane>> RoadNetwork::getLanes() {
    std::vector<std::shared_ptr<Lane>> collectedLanes;
    for (const auto &containedLanes : lanes) {
        collectedLanes.push_back(containedLanes.second.second);
    }
    return collectedLanes;
}

const std::vector<std::shared_ptr<Intersection>> &RoadNetwork::getIntersections() const { return intersections; }

std::vector<std::shared_ptr<Lanelet>> RoadNetwork::findOccupiedLaneletsByShape(const polygon_type &polygonShape) {
    // find all relevant lanelets by making use of the rtree
    std::vector<value> relevantLanelets;
    pImpl->rtree.query(bgi::intersects(bg::return_envelope<box>(polygonShape.outer())),
                       std::back_inserter(relevantLanelets));
    std::vector<std::shared_ptr<Lanelet>> lanelets;
    lanelets.reserve(relevantLanelets.size());
    for (auto la : relevantLanelets)
        lanelets.push_back(findLaneletById(static_cast<size_t>(la.second)));

    // check intersection with relevant lanelets
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets;
    for (auto la : lanelets) {
        if (la->checkIntersection(polygonShape, ContainmentType::PARTIALLY_CONTAINED)) {
            occupiedLanelets.push_back(la);
        }
    }
    return occupiedLanelets;
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
        throw std::domain_error("RoadNetwork::findLaneletById: Lanelet with ID" + std::to_string(id) +
                                " does not exist in road network!");

    return *it;
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

std::vector<std::shared_ptr<Lane>> RoadNetwork::addLanes(const std::vector<std::shared_ptr<Lane>> &newLanes,
                                                         size_t initialLanelet) {
    std::vector<std::shared_ptr<Lane>> updatedLanes;
    for (const auto &la : newLanes) {
        if (lanes.count(la->getContainedLaneletIDs()) != 0u and
            lanes[la->getContainedLaneletIDs()].first.count(initialLanelet) != 0u) {
            updatedLanes.push_back(lanes.at(la->getContainedLaneletIDs()).second);
            continue;
        } else if (lanes.count(la->getContainedLaneletIDs()) != 0u) {
            lanes[la->getContainedLaneletIDs()].first.insert(initialLanelet);
            updatedLanes.push_back(lanes.at(la->getContainedLaneletIDs()).second);
        } else {
            lanes[la->getContainedLaneletIDs()] = {{initialLanelet}, la};
            updatedLanes.push_back(la);
        }
    }
    return updatedLanes;
}

std::vector<std::shared_ptr<Lane>> RoadNetwork::findLanesByBaseLanelet(size_t laneletID) {
    std::vector<std::shared_ptr<Lane>> relevantLanes;
    for (const auto &[laneIDs, laneMap] : lanes)
        if (laneIDs.count(laneletID) != 0u and laneMap.first.count(laneletID) != 0u)
            relevantLanes.push_back(laneMap.second);
    return relevantLanes;
}

std::vector<std::shared_ptr<Lane>> RoadNetwork::findLanesByContainedLanelet(size_t laneletID) {
    std::vector<std::shared_ptr<Lane>> relevantLanes;
    for (const auto &[laneIDs, laneMap] : lanes)
        if (laneIDs.count(laneletID) != 0u)
            relevantLanes.push_back(laneMap.second);
    return relevantLanes;
}
void RoadNetwork::setIdCounterRef(const std::shared_ptr<size_t> &idCounter) {
    if (idCounterRef == nullptr)
        idCounterRef = idCounter;
}

std::shared_ptr<size_t> RoadNetwork::getIdCounterRef() const { return idCounterRef; }
