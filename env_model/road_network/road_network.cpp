//
// Created by sebastian on 08.11.20.
//

#include "road_network.h"
#include "lanelet/lanelet_operations.h"

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
