//
// Created by sebastian on 13.11.20.
//

#ifndef ENV_MODEL_POSITION_PREDICATES_H
#define ENV_MODEL_POSITION_PREDICATES_H

#include "../obstacle/obstacle.h"
#include "../road_network/road_network.h"

class PositionPredicates {
    public:
        static bool onMainCarriageWay(int timeStep, const std::shared_ptr<Obstacle>& obstacle, const RoadNetwork& roadNetwork);

};


#endif //ENV_MODEL_POSITION_PREDICATES_H
