//
// Created by sebastian on 15.12.20.
//

#ifndef ENV_MODEL_BRAKING_PREDICATES_H
#define ENV_MODEL_BRAKING_PREDICATES_H

#include "../obstacle/obstacle.h"
#include "../roadNetwork/road_network.h"

class BrakingPredicates {
    public:
        explicit BrakingPredicates(std::shared_ptr<RoadNetwork> roadNetwork);
        void setRoadNetwork(const std::shared_ptr<RoadNetwork> &roadNetwork);

        static double safeDistance(double vFollow, double vLead, double aMinFollow, double aMinLead, double tReact);
        static bool keepsSafeDistancePrec(int timeStep, const std::shared_ptr<Obstacle>& vehicleFollow,
                                   const std::shared_ptr<Obstacle>& vehicleLead);

    private:
        std::shared_ptr<RoadNetwork> roadNetwork;
};


#endif //ENV_MODEL_BRAKING_PREDICATES_H
