//
// Created by Sebastian Maierhofer on 16.01.21.
//

#ifndef ENV_MODEL_PREDICATES_H
#define ENV_MODEL_PREDICATES_H

#include "../obstacle/obstacle.h"
#include "../roadNetwork/road_network.h"

class Predicates {
public:
    explicit Predicates(std::shared_ptr<RoadNetwork> roadNetwork);

    void setRoadNetwork(const std::shared_ptr<RoadNetwork> &roadNetwork);

    static double safeDistance(double vFollow, double vLead, double aMinFollow, double aMinLead, double tReact);

    static bool keepsSafeDistancePrec(int timeStep, const std::shared_ptr<Obstacle> &vehicleFollow,
                                      const std::shared_ptr<Obstacle> &vehicleLead);

    bool onMainCarriageWay(int timeStep, const std::shared_ptr<Obstacle> &obstacle);

    bool onMainCarriageWayRightLane(int timeStep, const std::shared_ptr<Obstacle> &obstacle);

    bool onAccessRamp(int timeStep, const std::shared_ptr<Obstacle> &obstacle);

    bool inFrontOf(int timeStep, const std::shared_ptr<Obstacle> &obs1, const std::shared_ptr<Obstacle> &obs2);

    bool inSameLane(int timeStep, const std::shared_ptr<Obstacle> &obs1, const std::shared_ptr<Obstacle> &obs2);

    bool cutIn(int timeStep, const std::shared_ptr<Obstacle> &obs1, const std::shared_ptr<Obstacle> &obs2);

    static bool reverse(int timeStep, const std::shared_ptr<Obstacle> &obs);

    bool makesUTurn(int timeStep, const  std::shared_ptr<Obstacle> &obs);

private:
    std::shared_ptr<RoadNetwork> roadNetwork;
};


#endif //ENV_MODEL_PREDICATES_H
