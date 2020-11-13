//
// Created by sebastian on 13.11.20.
//

#ifndef ENV_MODEL_POSITION_PREDICATES_H
#define ENV_MODEL_POSITION_PREDICATES_H

#include "../obstacle/obstacle.h"
#include "../road_network/road_network.h"

class PositionPredicates {
    public:
        bool onMainCarriageWay(int timeStep, std::shared_ptr<Obstacle> obstacle, RoadNetwork roadNetwork);

};


#endif //ENV_MODEL_POSITION_PREDICATES_H
