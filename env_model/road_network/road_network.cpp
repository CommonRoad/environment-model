//
// Created by sebastian on 08.11.20.
//

#include "road_network.h"

RoadNetwork::RoadNetwork(const std::vector<std::shared_ptr<Lanelet>> &network){
    laneletNetwork = network;
    createLanes(network);
}

const std::vector<std::shared_ptr<Lanelet>> &RoadNetwork::getLaneletNetwork() const {return laneletNetwork;}

void RoadNetwork::setLaneletNetwork(const std::vector<std::shared_ptr<Lanelet>> &network) {laneletNetwork = network;}

const std::vector<std::shared_ptr<Lane>> &RoadNetwork::getLanes() const {return lanes;}

void RoadNetwork::setLanes(const std::vector<std::shared_ptr<Lane>> &la) {RoadNetwork::lanes = la;}

void RoadNetwork::createLanes(const std::vector<std::shared_ptr<Lanelet>>& network) {
    std::vector<std::shared_ptr<Lane>> lanes;
    std::vector<std::shared_ptr<Lanelet>> startLanelets;
    std::vector<std::shared_ptr<Lanelet>> predecessors;
    LaneletType laneletType;
    for(const auto& la : network){
        if(la->getPredecessors().empty())
            startLanelets.push_back(la);
        else{
            predecessors.insert(predecessors.end(), la->getPredecessors().begin(), la->getPredecessors().end());
            laneletType = extractClassifyingLaneletType(la);
            bool breakLoop = false;
            for(const auto& pred : la->getPredecessors()){
                for(const auto& type : pred->getLaneletType()){
                    if(type != laneletType) {
                        startLanelets.push_back(la);
                        breakLoop = true;
                        break;
                    }
                }
                if(breakLoop)
                    break;
            }
        }
    }
    for(const auto& la : startLanelets){
        laneletType = extractClassifyingLaneletType(la);
        //merge lanelets
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
