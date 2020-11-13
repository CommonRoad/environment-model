//
// Created by sebastian on 08.11.20.
//

#ifndef ENV_MODEL_ROAD_NETWORK_H
#define ENV_MODEL_ROAD_NETWORK_H

#include "lanelet/lane.h"
#include "../obstacle/obstacle.h"

class RoadNetwork {
    public:
        explicit RoadNetwork(const std::vector<std::shared_ptr<Lanelet>> &laneletNetwork);

        [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getLaneletNetwork() const;
        [[nodiscard]] const std::vector<std::shared_ptr<Lane>> &getLanes() const;

        void setLaneletNetwork(const std::vector<std::shared_ptr<Lanelet>> &laneletNetwork);
        void setLanes(const std::vector<std::shared_ptr<Lane>> &lanes);
        std::vector<std::shared_ptr<Lanelet>> findOccupiedLaneletsByShape(const polygon_type &polygonShape);
        std::vector<std::shared_ptr<Lanelet>> getOccupiedLanelets(const std::shared_ptr<Obstacle>& obstacle, int timeStep);
        std::vector<std::shared_ptr<Lanelet>> findLaneletsByPosition(const std::vector<Lanelet> &lanelets, double xPos, double yPos);

    private:
        void createLanes(const std::vector<std::shared_ptr<Lanelet>>& laneletNetwork);

        std::vector<std::shared_ptr<Lanelet>> laneletNetwork;
        std::vector<std::shared_ptr<Lane>> lanes;

    static LaneletType extractClassifyingLaneletType(const std::shared_ptr<Lanelet> &la);
};


#endif //ENV_MODEL_ROAD_NETWORK_H
