//
// Created by sebastian on 08.11.20.
//

#ifndef ENV_MODEL_ROAD_NETWORK_H
#define ENV_MODEL_ROAD_NETWORK_H

#include "lanelet/lane.h"

class RoadNetwork {
    public:
        explicit RoadNetwork(const std::vector<std::shared_ptr<Lanelet>> &laneletNetwork);

        [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getLaneletNetwork() const;
        [[nodiscard]] const std::vector<std::shared_ptr<Lane>> &getLanes() const;

        void setLaneletNetwork(const std::vector<std::shared_ptr<Lanelet>> &laneletNetwork);
        void setLanes(const std::vector<std::shared_ptr<Lane>> &lanes);

    private:
        void createLanes(const std::vector<std::shared_ptr<Lanelet>> &laneletNetwork);

        std::vector<std::shared_ptr<Lanelet>> laneletNetwork;
        std::vector<std::shared_ptr<Lane>> lanes;
};


#endif //ENV_MODEL_ROAD_NETWORK_H
