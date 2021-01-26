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

    static bool inFrontOf(int timeStep, const std::shared_ptr<Obstacle> &obs1, const std::shared_ptr<Obstacle> &obs2);

    bool inSameLane(int timeStep, const std::shared_ptr<Obstacle> &obs1, const std::shared_ptr<Obstacle> &obs2);

    bool cutIn(int timeStep, const std::shared_ptr<Obstacle> &obs1, const std::shared_ptr<Obstacle> &obs2);

    static bool reverse(int timeStep, const std::shared_ptr<Obstacle> &obs);

    bool makesUTurn(int timeStep, const  std::shared_ptr<Obstacle> &obs);

    static bool inStandstill(int timeStep, const  std::shared_ptr<Obstacle> &obs);

    bool stopLineInFront(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether the kth vehicle is left of the pth vehicle
     *
     * @param timeStep Time step of interest.
     * @param obsP The pth obstacle.
     * @param obsK The kth obstacle.
     * @return Boolean indicating satisfaction
     */
    static bool onLeftIncoming(int timeStep, const std::shared_ptr<Obstacle> &obsP, const std::shared_ptr<Obstacle> &obsK);

    /**
     * Evaluates whether the intersection the obstacle belongs to is regulated by traffic signs.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    static bool intersectionRegulatedByTrafficSigns(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether the intersection the obstacle belongs to is regulated by traffic lights.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    static bool intersectionRegulatedByTrafficLights(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether two vehicles are part of the same intersection.
     *
     * @param timeStep Time step of interest.
     * @param obsP The pth obstacle.
     * @param obsK The kth obstacle.
     * @return Boolean indicating satisfaction.
     */
    static bool inSameIntersection(int timeStep, const std::shared_ptr<Obstacle> &obsP, const std::shared_ptr<Obstacle> &obsK);

    /**
     * Evaluates whether a vehicle is within the main area of an intersection.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    bool inIntersectionMainArea(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether a vehicle is on an incoming lanelet of an intersection.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    bool onIncoming(int timeStep, const std::shared_ptr<Obstacle> &obs);

private:
    std::shared_ptr<RoadNetwork> roadNetwork;
};


#endif //ENV_MODEL_PREDICATES_H
