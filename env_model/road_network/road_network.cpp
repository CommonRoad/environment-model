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

void RoadNetwork::createLanes(const std::vector<std::shared_ptr<Lanelet>> &network) {
    std::vector<std::shared_ptr<Lane>> lanes;
    std::vector<std::shared_ptr<Lanelet>> startLanelets;
    LaneletType laneletType;
    for(const auto& la : network){
        if(la->getPredecessors().empty())
            startLanelets.push_back(la);
        else{

        }
    }
    for(const auto& la : startLanelets){
        // find classifying lanelet type for lane
        for(const auto& type : la->getLaneletType()){
            if(type == LaneletType::accessRamp)
                laneletType = LaneletType::accessRamp;
            else if(type == LaneletType::exitRamp)
                laneletType = LaneletType::exitRamp;
            else if(type == LaneletType::mainCarriageWay)
                laneletType = LaneletType::mainCarriageWay;
            else if(type == LaneletType::shoulder)
                laneletType = LaneletType::shoulder;
        }
        //merge lanelets
    }
}
