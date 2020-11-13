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
        static std::vector<std::shared_ptr<Lanelet>> findOccupiedLaneletsByShape(std::vector<std::shared_ptr<Lanelet>> lanelets, const polygon_type &polygonShape);
        static std::vector<std::shared_ptr<Lanelet>> findLaneletsByPosition(const std::vector<std::shared_ptr<Lanelet>> &lanelets, double xPos, double yPos);

    private:
        void createLanes(const std::vector<std::shared_ptr<Lanelet>>& laneletNetwork);

        std::vector<std::shared_ptr<Lanelet>> laneletNetwork;
        std::vector<std::shared_ptr<Lane>> lanes;

    static LaneletType extractClassifyingLaneletType(const std::shared_ptr<Lanelet> &la);
};


#endif //ENV_MODEL_ROAD_NETWORK_H
