#include <utility>

#include <boost/version.hpp>
#if (BOOST_VERSION / 100000) == 1 && (BOOST_VERSION / 100 % 1000) < 78 // Minor version < 78
// On Boost<1.78.0, the cartesian blanket header does not exist
#include <boost/geometry/strategies/strategies.hpp>

// On Boost<1.78.0, this needs to be included before <boost/geometry/index/parameters.hpp>
#include <boost/geometry/strategies/default_strategy.hpp>
#else
#include <boost/geometry/strategies/cartesian.hpp>
#endif

#include <boost/geometry/index/parameters.hpp>
#include <boost/geometry/index/rtree.hpp>

#include "commonroad_cpp/roadNetwork/intersection/incoming_group.h"
#include <commonroad_cpp/auxiliaryDefs/regulatory_elements.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>
#include <commonroad_cpp/roadNetwork/road_network.h>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

struct RoadNetwork::impl {
    bgi::rtree<value, bgi::quadratic<16>>
        rtree; //**< rtree defined by lanelets of road network for faster occupancy calculation*/
};

RoadNetwork::RoadNetwork(RoadNetwork &&) noexcept = default;

RoadNetwork::~RoadNetwork() = default;

RoadNetwork &RoadNetwork::operator=(RoadNetwork &&) noexcept = default;

RoadNetwork::RoadNetwork(const std::vector<std::shared_ptr<Lanelet>> &network, const SupportedTrafficSignCountry cou,
                         std::vector<std::shared_ptr<TrafficSign>> signs,
                         std::vector<std::shared_ptr<TrafficLight>> lights,
                         std::vector<std::shared_ptr<Intersection>> inters)
    : laneletNetwork(network), country(cou), trafficSigns(std::move(signs)), trafficLights(std::move(lights)),
      intersections(std::move(inters)), pImpl(std::make_unique<impl>()) {
    // construct Rtree out of lanelets
    for (const std::shared_ptr<Lanelet> &let : network)
        pImpl->rtree.insert(std::make_pair(let->getBoundingBox(), let->getId()));
    trafficSignIDLookupTable = TrafficSignLookupTableByCountry.at(cou);
}

const std::vector<std::shared_ptr<Lanelet>> &RoadNetwork::getLaneletNetwork() const { return laneletNetwork; }

const std::vector<std::shared_ptr<TrafficSign>> &RoadNetwork::getTrafficSigns() const { return trafficSigns; }

const std::vector<std::shared_ptr<TrafficLight>> &RoadNetwork::getTrafficLights() const { return trafficLights; }

std::vector<std::shared_ptr<Lane>> RoadNetwork::getLanes() const {
    std::vector<std::shared_ptr<Lane>> collectedLanes;
    for (const auto &[fst, snd] : lanes) {
        collectedLanes.push_back(snd.second);
    }
    return collectedLanes;
}

const std::vector<std::shared_ptr<Intersection>> &RoadNetwork::getIntersections() const { return intersections; }

std::vector<std::shared_ptr<Lanelet>> RoadNetwork::findOccupiedLaneletsByShape(const multi_polygon_type &polygonShape) {
    // Collect RTree hits across all sub-polygons; deduplicate by ID so each
    // lanelet undergoes the expensive polygon intersection test at most once.
    tsl::robin_map<size_t, bool> seenIds;
    std::vector<std::shared_ptr<Lanelet>> candidates;
    for (const auto &polygon : polygonShape) {
        std::vector<value> hits;
        pImpl->rtree.query(bgi::intersects(bg::return_envelope<box>(polygon.outer())), std::back_inserter(hits));
        for (const auto &[box, id] : hits) {
            if (seenIds.emplace(id, true).second)
                candidates.push_back(findLaneletById(id));
        }
    }

    // Full polygon intersection test on deduplicated candidates.
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets;
    for (const auto &let : candidates) {
        for (const auto &polygon : polygonShape) {
            if (let->checkIntersection(polygon, ContainmentType::PARTIALLY_CONTAINED)) {
                occupiedLanelets.push_back(let);
                break;
            }
        }
    }
    return occupiedLanelets;
}

std::vector<std::shared_ptr<Lanelet>> RoadNetwork::findLaneletsByPosition(const double xPos, const double yPos) {
    std::vector<Lanelet> lanelet;
    polygon_type polygonPos;
    bg::append(polygonPos, point_type{xPos, yPos});

    return findOccupiedLaneletsByShape({polygonPos});
}

std::shared_ptr<Lanelet> RoadNetwork::findLaneletById(size_t laneletID) {
    // Lazily populate the index on first miss to avoid touching every constructor / setter.
    if (laneletByIdIndex_.empty() && !laneletNetwork.empty()) {
        laneletByIdIndex_.reserve(laneletNetwork.size());
        for (const auto &la : laneletNetwork)
            laneletByIdIndex_.emplace(la->getId(), la);
    }
    const auto it = laneletByIdIndex_.find(laneletID);
    if (it == laneletByIdIndex_.end())
        throw std::domain_error("RoadNetwork::findLaneletById: Lanelet with ID " + std::to_string(laneletID) +
                                " does not exist in road network!");
    return it->second;
}

std::shared_ptr<TrafficLight> RoadNetwork::findTrafficLightById(size_t lightID) {
    // Lazily build an O(1) index on first call.
    if (trafficLightByIdIndex_.empty() && !trafficLights.empty()) {
        trafficLightByIdIndex_.reserve(trafficLights.size());
        for (const auto &tl : trafficLights)
            trafficLightByIdIndex_.emplace(tl->getId(), tl);
    }
    const auto it = trafficLightByIdIndex_.find(lightID);
    if (it == trafficLightByIdIndex_.end())
        throw std::domain_error("RoadNetwork::findTrafficLightById: Traffic light with ID " + std::to_string(lightID) +
                                " does not exist in road network!");
    return it->second;
}

SupportedTrafficSignCountry RoadNetwork::getCountry() const { return country; }

SupportedTrafficSignCountry RoadNetwork::matchStringToCountry(const std::string &name) {
    if (name == "DEU")
        return SupportedTrafficSignCountry::GERMANY;
    if (name == "USA")
        return SupportedTrafficSignCountry::USA;
    if (name == "ESP")
        return SupportedTrafficSignCountry::SPAIN;
    if (name == "ARG")
        return SupportedTrafficSignCountry::ARGENTINA;
    if (name == "BEL")
        return SupportedTrafficSignCountry::BELGIUM;
    if (name == "AUS")
        return SupportedTrafficSignCountry::AUSTRALIA;
    return SupportedTrafficSignCountry::ZAMUNDA;
}

std::string RoadNetwork::extractTrafficSignIDForCountry(const TrafficSignTypes type) const {
    return trafficSignIDLookupTable->at(type);
}

std::vector<std::shared_ptr<Lane>> RoadNetwork::addLanes(const std::vector<std::shared_ptr<Lane>> &newLanes,
                                                         size_t initialLanelet) {
    std::vector<std::shared_ptr<Lane>> updatedLanes;
    for (const auto &lane : newLanes) {
        // Single find() call instead of count()+find(); maintain reverse indices.
        const auto &containedIds = lane->getContainedLaneletIDs();
        if (const auto it = lanes.find(containedIds); it != lanes.end()) {
            if (it->second.first.count(initialLanelet) != 0u) {
                // Already registered with this base lanelet — return existing.
                updatedLanes.push_back(it->second.second);
            } else {
                // Existing lane, new base lanelet.
                it->second.first.insert(initialLanelet);
                updatedLanes.push_back(it->second.second);
                lanesByBaseLaneletIndex_[initialLanelet].push_back(it->second.second);
            }
        } else {
            // Truly new lane — register and update both reverse indices.
            lanes[containedIds] = {{initialLanelet}, lane};
            updatedLanes.push_back(lane);
            for (const auto &id : containedIds)
                lanesByContainedLaneletIndex_[id].push_back(lane);
            lanesByBaseLaneletIndex_[initialLanelet].push_back(lane);
        }
    }
    return updatedLanes;
}

std::vector<std::shared_ptr<Lane>> RoadNetwork::findLanesByBaseLanelet(const size_t laneletID) {
    // O(1) index lookup instead of O(n) scan.
    const auto it = lanesByBaseLaneletIndex_.find(laneletID);
    return it != lanesByBaseLaneletIndex_.end() ? it->second : std::vector<std::shared_ptr<Lane>>{};
}

std::vector<std::shared_ptr<Lane>> RoadNetwork::findLanesByContainedLanelet(const size_t laneletID) {
    // O(1) index lookup instead of O(n) scan.
    const auto it = lanesByContainedLaneletIndex_.find(laneletID);
    return it != lanesByContainedLaneletIndex_.end() ? it->second : std::vector<std::shared_ptr<Lane>>{};
}
void RoadNetwork::setIdCounterRef(const std::shared_ptr<size_t> &idCounter) {
    if (idCounterRef == nullptr)
        idCounterRef = idCounter;
}

std::shared_ptr<size_t> RoadNetwork::getIdCounterRef() const { return idCounterRef; }

std::shared_ptr<IncomingGroup> RoadNetwork::findIncomingGroupByLanelet(const std::shared_ptr<Lanelet> &lanelet) const {
    // Build lanelet-id → incoming-group index once, then O(1) lookup.
    if (incomingByLaneletIndex_.empty() && !intersections.empty()) {
        for (const auto &inter : intersections)
            for (const auto &incom : inter->getIncomingGroups())
                for (const auto &let : incom->getIncomingLanelets())
                    incomingByLaneletIndex_.emplace(let->getId(), incom);
    }
    const auto it = incomingByLaneletIndex_.find(lanelet->getId());
    return it != incomingByLaneletIndex_.end() ? it->second : nullptr;
}

std::shared_ptr<IncomingGroup>
RoadNetwork::findIncomingGroupByOutgoingGroup(const std::shared_ptr<OutgoingGroup> &outgoingGroup) const {
    // Build outgoing-group-id → incoming-group index once, then O(1) lookup.
    if (incomingByOutgoingGroupIndex_.empty() && !intersections.empty()) {
        for (const auto &inter : intersections)
            for (const auto &incom : inter->getIncomingGroups())
                if (incom->getOutgoingGroupID().has_value())
                    incomingByOutgoingGroupIndex_.emplace(incom->getOutgoingGroupID().value(), incom);
    }
    const auto it = incomingByOutgoingGroupIndex_.find(outgoingGroup->getId());
    return it != incomingByOutgoingGroupIndex_.end() ? it->second : nullptr;
}

std::shared_ptr<OutgoingGroup> RoadNetwork::findOutgoingGroupByLanelet(const std::shared_ptr<Lanelet> &lanelet) const {
    // Build lanelet-id → outgoing-group index once, then O(1) lookup.
    if (outgoingByLaneletIndex_.empty() && !intersections.empty()) {
        for (const auto &inter : intersections)
            for (const auto &out : inter->getOutgoingGroups())
                for (const auto &let : out->getOutgoingLanelets())
                    outgoingByLaneletIndex_.emplace(let->getId(), out);
    }
    const auto it = outgoingByLaneletIndex_.find(lanelet->getId());
    return it != outgoingByLaneletIndex_.end() ? it->second : nullptr;
}

const std::vector<std::shared_ptr<Intersection>> &
RoadNetwork::findIntersectionsByLaneletId(const size_t laneletId, const std::shared_ptr<RoadNetwork> &self) const {
    // Build lanelet-id → intersections index once using the existing getMemberLanelets() cache.
    if (!intersectionsByLaneletIdBuilt_) {
        for (const auto &inter : intersections)
            for (const auto &let : inter->getMemberLanelets(self))
                intersectionsByLaneletIdIndex_[let->getId()].push_back(inter);
        intersectionsByLaneletIdBuilt_ = true;
    }
    static const std::vector<std::shared_ptr<Intersection>> empty;
    const auto it = intersectionsByLaneletIdIndex_.find(laneletId);
    return it != intersectionsByLaneletIdIndex_.end() ? it->second : empty;
}

const std::shared_ptr<LaneletGraph> &RoadNetwork::getTopologicalMap() const {
    if (topologicalMap != nullptr)
        return topologicalMap;
    topologicalMap = std::make_shared<LaneletGraph>(laneletNetwork);
    return topologicalMap;
}

const std::shared_ptr<Intersection> &RoadNetwork::getIntersectionByID(const size_t intersectionID) const {
    // Lazily populate the index on first miss to avoid touching every constructor / setter.
    if (intersectionByIdIndex_.empty() && !intersections.empty()) {
        intersectionByIdIndex_.reserve(intersections.size());
        for (const auto &inter : intersections)
            intersectionByIdIndex_.emplace(inter->getId(), inter);
    }
    const auto it = intersectionByIdIndex_.find(intersectionID);
    if (it == intersectionByIdIndex_.end())
        throw std::domain_error("RoadNetwork::getIntersectionByID: Intersection with ID " +
                                std::to_string(intersectionID) + " does not exist in road network!");
    return it->second;
}
