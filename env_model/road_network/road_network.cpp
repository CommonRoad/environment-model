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

std::vector<std::shared_ptr<Lanelet>> RoadNetwork::findOccupiedLaneletsByShape(const polygon_type &polygonShape) {

    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets;

#pragma omp parallel for schedule(guided)
    for (size_t i = 0; i < laneletNetwork.size(); i++) {
        if (laneletNetwork[i]->checkIntersection(polygonShape, PARTIALLY_CONTAINED)) {
#pragma omp critical
            occupiedLanelets.push_back(laneletNetwork[i]);
        }
    }

    return occupiedLanelets;
}

std::vector<std::shared_ptr<Lanelet>> RoadNetwork::getOccupiedLanelets(const std::shared_ptr<Obstacle>& obstacle, int timeStep) {
    // get lanelets which intersect with shape of ego vehicle
    polygon_type polygonShape{obstacle->getOccupancyPolygonShape(timeStep)};
    std::vector<std::shared_ptr<Lanelet>> occupied{RoadNetwork::findOccupiedLaneletsByShape(polygonShape)};

    return occupied;

}

std::vector<std::shared_ptr<Lanelet>> RoadNetwork::findLaneletsByPosition(const std::vector<Lanelet> &lanelets, double xPos, double yPos) {

    std::vector<Lanelet> lanelet;
    polygon_type polygonPos;
    bg::append(polygonPos, point_type{xPos, yPos});

    return findOccupiedLaneletsByShape(polygonPos);
}

std::shared_ptr<Lanelet> findLaneletsById(std::vector<std::shared_ptr<Lanelet>> lanelets, size_t id) {
    auto it = std::find_if(std::begin(lanelets), std::end(lanelets), [id](auto val) { return val->getId() == id; });
    if (it == std::end(lanelets)) {
        throw std::domain_error(std::to_string(id));
    }
    return *it;
}
