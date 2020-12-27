//
// Created by Sebastian Maierhofer on 13.11.20.
//

#ifndef ENV_MODEL_POSITION_PREDICATES_H
#define ENV_MODEL_POSITION_PREDICATES_H

#include "../obstacle/obstacle.h"
#include "../roadNetwork/road_network.h"

class PositionPredicates {
    public:
        explicit PositionPredicates(std::shared_ptr<RoadNetwork> roadNetwork);
        void setRoadNetwork(const std::shared_ptr<RoadNetwork> &roadNetwork);

        bool onMainCarriageWay(int timeStep, const std::shared_ptr<Obstacle>& obstacle);
        bool onMainCarriageWayRightLane(int timeStep, const std::shared_ptr<Obstacle>& obstacle);
        bool onAccessRamp(int timeStep, const std::shared_ptr<Obstacle>& obstacle);
        static bool inFrontOf(int timeStep, const std::shared_ptr<Obstacle>& obs1, const std::shared_ptr<Obstacle>& obs2);

    private:
        std::shared_ptr<RoadNetwork> roadNetwork;
};


#endif //ENV_MODEL_POSITION_PREDICATES_H
