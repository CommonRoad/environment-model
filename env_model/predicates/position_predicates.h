//
// Created by sebastian on 13.11.20.
//

#ifndef ENV_MODEL_POSITION_PREDICATES_H
#define ENV_MODEL_POSITION_PREDICATES_H

#include "../obstacle/obstacle.h"
#include "../road_network/road_network.h"

class PositionPredicates {
    public:
        bool onMainCarriageWay(int timeStep, const std::shared_ptr<Obstacle>& obstacle);
        bool onMainCarriageWayRightLane(int timeStep, const std::shared_ptr<Obstacle>& obstacle);
        bool onAccessRamp(int timeStep, const std::shared_ptr<Obstacle>& obstacle);
        bool inFrontOf(int timeStep, const std::shared_ptr<Obstacle>& obs1, const std::shared_ptr<Obstacle>& obs2);

        void setRoadNetwork(const std::shared_ptr<RoadNetwork> &roadNetwork);

    PositionPredicates(std::shared_ptr<RoadNetwork> roadNetwork);

private:
        std::shared_ptr<RoadNetwork> roadNetwork;

};


#endif //ENV_MODEL_POSITION_PREDICATES_H
