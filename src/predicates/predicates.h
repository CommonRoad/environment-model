//
// Created by Sebastian Maierhofer on 16.01.21.
//

#ifndef ENV_MODEL_PREDICATES_H
#define ENV_MODEL_PREDICATES_H

#include "../obstacle/obstacle.h"
#include "../roadNetwork/road_network.h"

class Predicates {
public:
    explicit Predicates(std::shared_ptr<RoadNetwork> roadNetwork, SupportedTrafficSignCountry country = SupportedTrafficSignCountry::GERMANY);

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
     * Evaluates whether a vehicle is within the main area of an intersection.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    bool inIntersectionMainArea(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Returns set of pointers to active traffic lights of lanelets a given obstacle occupies.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    std::set<std::shared_ptr<TrafficLight>> activeTrafficLights(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether an obstacle is on an lanelet which is regulated by an green arrow traffic sign.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    bool atGreenArrow(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether the obstacle is on an lanelet regulated by a traffic light for straight direction which is currently red.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    bool atRedStraightTrafficLight(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether the obstacle is on an lanelet regulated by a traffic light for lt direction which is currently red.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    bool atRedLeftTrafficLight(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether the obstacle is on an lanelet regulated by a traffic light for right direction which is currently red.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    bool atRedRightTrafficLight(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether the obstacle is on an lanelet regulated by a traffic light for a provided turning direction which is currently red.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @param turnDir Turning direction the red traffic light should have.
     * @return Boolean indicating satisfaction.
     */
    bool atRedTrafficLight(int timeStep, const std::shared_ptr<Obstacle> &obs, TrafficLightDirection turnDir);

    /**
     * Evaluates whether the obstacle is on an right outgoing lanelet of an intersection incoming.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    bool onRightOutgoing(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether the obstacle is on an left outgoing lanelet of an intersection incoming.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    bool onLeftOutgoing(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether the obstacle is on an straight outgoing lanelet of an intersection incoming.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    bool onStraightOutgoing(int timeStep, const std::shared_ptr<Obstacle> &obs);

private:
    std::shared_ptr<RoadNetwork> roadNetwork;
    SupportedTrafficSignCountry country;
};


#endif //ENV_MODEL_PREDICATES_H
