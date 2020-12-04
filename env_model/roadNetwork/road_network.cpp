//
// Created by sebastian on 08.11.20.
//

#include "road_network.h"

#include <utility>
#include "lanelet/lanelet_operations.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

RoadNetwork::RoadNetwork(const std::vector<std::shared_ptr<Lanelet>> &network){
    laneletNetwork = network;
    for(const std::shared_ptr<Lanelet>& la : network){
        box b = bg::return_envelope<box>(la->getOuterPolygon());
        rtree.insert(std::make_pair(b, la->getId()));
    }
    createLanes(network);
}

const std::vector<std::shared_ptr<Lanelet>> &RoadNetwork::getLaneletNetwork() const {return laneletNetwork;}

void RoadNetwork::setLaneletNetwork(const std::vector<std::shared_ptr<Lanelet>> &network) {laneletNetwork = network;}

std::vector<std::shared_ptr<Lane>> RoadNetwork::getLanes() {return lanes;}

void RoadNetwork::setLanes(std::vector<std::shared_ptr<Lane>> la) {lanes = std::move(la);}


void RoadNetwork::createLanes(const std::vector<std::shared_ptr<Lanelet>>& network) {
    std::vector<std::shared_ptr<Lanelet>> startLanelets;

    LaneletType laneletType;
    for(const auto& la : network){
        if(la->getPredecessors().empty())
            startLanelets.push_back(la);
        else{
            std::vector<std::shared_ptr<Lanelet>> predecessors;
            for(const std::shared_ptr<Lanelet>& pre : la->getPredecessors()){
                predecessors.push_back(pre);
            }
            laneletType = extractClassifyingLaneletType(la);
            for(const auto& pred : la->getPredecessors()){
                if(!std::any_of(pred->getLaneletType().begin(), pred->getLaneletType().end(), [laneletType](LaneletType t){return t == laneletType;})){
                    startLanelets.push_back(la);
                    break;
                }
            }
        }
    }
    for(const auto& la : startLanelets){
        laneletType = extractClassifyingLaneletType(la);
        lanes.push_back(combineLaneletAndSuccessorsWithSameTypeToLane(la, laneletType));
    }
}

LaneletType RoadNetwork::extractClassifyingLaneletType(const std::shared_ptr<Lanelet> &la) {
    for(const auto& type : la->getLaneletType()){
        if(type == LaneletType::accessRamp)
            return LaneletType::accessRamp;
        else if(type == LaneletType::exitRamp)
            return LaneletType::exitRamp;
        else if(type == LaneletType::mainCarriageWay)
            return LaneletType::mainCarriageWay;
        else if(type == LaneletType::shoulder)
            return LaneletType::shoulder;
    }
    return LaneletType::mainCarriageWay;
}


std::vector<std::shared_ptr<Lanelet>> RoadNetwork::findOccupiedLaneletsByShape(const polygon_type &polygonShape) {
    box query_box(point(0, 0), point(5, 5));
    std::vector<value> relevantLanlets;
    rtree.query(bgi::intersects(bg::return_envelope<box>(polygonShape.outer())), std::back_inserter(relevantLanlets));
    std::vector<std::shared_ptr<Lanelet>> lanelets;
    for(auto la : relevantLanlets)
        lanelets.push_back(findLaneletById(la.second));
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets;
//#pragma omp parallel for schedule(guided)
    for (size_t i = 0; i < lanelets.size(); i++) {
        if (lanelets[i]->checkIntersection(polygonShape, PARTIALLY_CONTAINED)) {
//#pragma omp critical
            occupiedLanelets.push_back(lanelets[i]);
        }
    }

    return occupiedLanelets;
}

std::shared_ptr<Lane> RoadNetwork::findLaneByShape(std::vector<std::shared_ptr<Lane>> possibleLanes, const polygon_type &polygonShape) {

//#pragma omp parallel for schedule(guided)
    for (size_t i = 0; i < possibleLanes.size(); i++) {
        if (possibleLanes[i]->checkIntersection(polygonShape, PARTIALLY_CONTAINED)) {
//#pragma omp critical
            return possibleLanes[i];
        }
    }
}

std::vector<std::shared_ptr<Lanelet>> RoadNetwork::findLaneletsByPosition(double xPos, double yPos) {

    std::vector<Lanelet> lanelet;
    polygon_type polygonPos;
    bg::append(polygonPos, point_type{xPos, yPos});

    return RoadNetwork::findOccupiedLaneletsByShape(polygonPos);
}

std::shared_ptr<Lanelet> RoadNetwork::findLaneletById(size_t id) {
    auto it = std::find_if(std::begin(laneletNetwork), std::end(laneletNetwork), [id](auto val) { return val->getId() == id; });
    if (it == std::end(laneletNetwork)) {
        throw std::domain_error(std::to_string(id));
    }
    return *it;
}
