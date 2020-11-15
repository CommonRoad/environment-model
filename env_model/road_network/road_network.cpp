//
// Created by sebastian on 08.11.20.
//

#include "road_network.h"
#include "lanelet/lanelet_operations.h"
#include "boost/geometry.hpp"

namespace bg = boost::geometry;

RoadNetwork::RoadNetwork(const std::vector<std::shared_ptr<Lanelet>> &network){
    laneletNetwork = network;
    createLanes(network);
}

const std::vector<std::shared_ptr<Lanelet>> &RoadNetwork::getLaneletNetwork() const {return laneletNetwork;}

void RoadNetwork::setLaneletNetwork(const std::vector<std::shared_ptr<Lanelet>> &network) {laneletNetwork = network;}

const std::vector<std::shared_ptr<Lane>> &RoadNetwork::getLanes() const {return lanes;}

void RoadNetwork::setLanes(const std::vector<std::shared_ptr<Lane>> &la) {RoadNetwork::lanes = la;}

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
        lanes.push_back(std::make_shared<Lane>(combineLaneletAndSuccessorsWithSameTypeToLane(la, laneletType)));
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

std::vector<std::shared_ptr<Lanelet>> RoadNetwork::findOccupiedLaneletsByShape(std::vector<std::shared_ptr<Lanelet>> lanelets, const polygon_type &polygonShape) {

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

std::shared_ptr<Lane> RoadNetwork::findLaneByShape(std::shared_ptr<Lane> possibleLanes, const polygon_type &polygonShape) {

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

    return RoadNetwork::findOccupiedLaneletsByShape(laneletNetwork, polygonPos);
}

std::shared_ptr<Lanelet> RoadNetwork::findLaneletById(size_t id) {
    auto it = std::find_if(std::begin(laneletNetwork), std::end(laneletNetwork), [id](auto val) { return val->getId() == id; });
    if (it == std::end(laneletNetwork)) {
        throw std::domain_error(std::to_string(id));
    }
    return *it;
}

std::shared_ptr<Lane> RoadNetwork::findLaneById(size_t id) {
    auto it = std::find_if(std::begin(lanes), std::end(lanes), [id](auto val) { return val->getId() == id; });
    if (it == std::end(lanes)) {
        throw std::domain_error(std::to_string(id));
    }
    return *it;
}
