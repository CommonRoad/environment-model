//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "road_network.h"

#include <commonroad_cpp/auxiliaryDefs/traffic_signs.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <utility>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

RoadNetwork::RoadNetwork(const std::vector<std::shared_ptr<Lanelet>> &network, SupportedTrafficSignCountry cou,
                         std::vector<std::shared_ptr<TrafficSign>> signs,
                         std::vector<std::shared_ptr<TrafficLight>> lights,
                         std::vector<std::shared_ptr<Intersection>> inters)
    : laneletNetwork(network), country(cou), trafficSigns(std::move(signs)), trafficLights(std::move(lights)),
      intersections(std::move(inters)) {
    // construct Rtree out of lanelets
    for (const std::shared_ptr<Lanelet> &la : network)
        rtree.insert(std::make_pair(la->getBoundingBox(), la->getId()));
    trafficSignIDLookupTable = TrafficSignLookupTableByCountry.at(cou);
}

const std::vector<std::shared_ptr<Lanelet>> &RoadNetwork::getLaneletNetwork() const { return laneletNetwork; }

std::vector<std::shared_ptr<Lane>> RoadNetwork::getLanes() { return lanes; }

const std::vector<std::shared_ptr<Intersection>> &RoadNetwork::getIntersections() const { return intersections; }

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
        throw std::domain_error("RoadNetwork::findLaneletById: Lanelet with ID" + std::to_string(id) +
                                " does not exist in road netowrk!");

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
