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

    /**
     * Setter for road network.
     * @param roadNetwork Pointer to road network.
     */
    void setRoadNetwork(const std::shared_ptr<RoadNetwork> &roadNetwork);

    /**
     * Computes the safe distance between a following and leading obstacle.
     *
     * @param vFollow Velocity of following obstacle.
     * @param vLead Velocity of leading obstacle.
     * @param aMinFollow Minimum acceleration of following obstacle.
     * @param aMinLead Minimum acceleration of leading obstacle.
     * @param tReact Reaction time of obstacle.
     * @return Safe distance of following vehicle with respect to the leading vehicle.
     */
    static double safeDistance(double vFollow, double vLead, double aMinFollow, double aMinLead, double tReact);

    /**
     * Evaluates whether the kth obstacle maintains the safe distance to the pth vehicle.
     *
     * @param timeStep Time step of interest.
     * @param obsK The kth obstacle.
     * @param obsK The pth obstacle.
     * @return Boolean indicating satisfaction.
     */
    static bool keepsSafeDistancePrec(int timeStep, const std::shared_ptr<Obstacle> &obsK,
                                      const std::shared_ptr<Obstacle> &obsP);

    /**
     * Evaluates whether the obstacle is on the main carriageway.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    bool onMainCarriageWay(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether the obstacle is on the main carriageway right lane.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    bool onMainCarriageWayRightLane(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether the obstacle is on an access ramp.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    bool onAccessRamp(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether the kth obstacle is in front of the pth obstacle.
     *
     * @param timeStep Time step of interest.
     * @param obsK The kth obstacle.
     * @param obsK The pth obstacle.
     * @return Boolean indicating satisfaction.
     */
    static bool inFrontOf(int timeStep, const std::shared_ptr<Obstacle> &obsK, const std::shared_ptr<Obstacle> &obsP);

    /**
     * Evaluates whether two obstacles are in the same lane.
     *
     * @param timeStep Time step of interest.
     * @param obsK The kth obstacle.
     * @param obsK The pth obstacle.
     * @return Boolean indicating satisfaction.
     */
    bool inSameLane(int timeStep, const std::shared_ptr<Obstacle> &obsK, const std::shared_ptr<Obstacle> &obsP);

    /**
     * Evaluates whether the kth obstacle performs a cut-in into the lane of the pth obstacle.
     *
     * @param timeStep Time step of interest.
     * @param obsK The kth obstacle.
     * @param obsK The pth obstacle.
     * @return Boolean indicating satisfaction.
     */
    bool cutIn(int timeStep, const std::shared_ptr<Obstacle> &obsK, const std::shared_ptr<Obstacle> &obsP);

    /**
     * Evaluates whether a vehicle reverses.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    static bool reverse(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether a vehicle makes a u-turn.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    bool makesUTurn(int timeStep, const  std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether a vehicle is in standstill.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    static bool inStandstill(int timeStep, const  std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether a vehicle has a stop line in front.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
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
     * Evaluates whether an obstacle is on a lanelet which is regulated by an green arrow traffic sign.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    bool atGreenArrow(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether the obstacle is on a lanelet regulated by a traffic light for straight direction which is currently red.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    bool atRedStraightTrafficLight(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether the obstacle is on a lanelet regulated by a traffic light for lt direction which is currently red.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    bool atRedLeftTrafficLight(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether the obstacle is on a lanelet regulated by a traffic light for right direction which is currently red.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    bool atRedRightTrafficLight(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether the obstacle is on a lanelet regulated by a traffic light for a provided turning direction which is currently red.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @param turnDir Turning direction the red traffic light should have.
     * @return Boolean indicating satisfaction.
     */
    bool atRedTrafficLight(int timeStep, const std::shared_ptr<Obstacle> &obs, TurningDirections turnDir);

    /**
     * Evaluates whether the obstacle is on a right outgoing lanelet of an intersection incoming.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    bool onRightOutgoing(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether the obstacle is on a left outgoing lanelet of an intersection incoming.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    bool onLeftOutgoing(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether the obstacle is on a straight outgoing lanelet of an intersection incoming.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    bool onStraightOutgoing(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether the obstacle is on an incoming lanelet of an intersection.
     *
     * @param timeStep Time step of interest.
     * @param obs The obstacle for which the predicate should be evaluated.
     * @return Boolean indicating satisfaction.
     */
    bool onIncoming(int timeStep, const std::shared_ptr<Obstacle> &obs);

    /**
     * Evaluates whether the kth obstacle has priority over the pth obstacle at an intersection.
     *
     * @param timeStep Time step of interest.
     * @param obsK The kth obstacle.
     * @param obsK The pth obstacle.
     * @return Boolean indicating satisfaction.
     */
    bool hasPriority(int timeStep, const std::shared_ptr<Obstacle> &obsK, const std::shared_ptr<Obstacle> &obsP, TurningDirections dirK, TurningDirections dirP);

    int getPriority(int timeStep, const std::shared_ptr<Obstacle> &obs, TurningDirections dir);

    static bool causesBraking(int timeStep, const std::shared_ptr<Obstacle> &obsK, const std::shared_ptr<Obstacle> &obsP);

    static std::vector<std::shared_ptr<Lanelet>> incomingLaneletOfLanelet(const std::shared_ptr<Lanelet>& la);

    static std::string extractPriorityTrafficSign(const std::shared_ptr<Lanelet>& lanelet);

    static std::string extractPriorityTrafficSignId(const std::vector<std::shared_ptr<Lanelet>>& lanelets);

    static std::vector<std::shared_ptr<Lanelet>> findUpcomingIncomingLanelets(const std::vector<std::shared_ptr<Lanelet>>& lanelets);

    static std::vector<std::shared_ptr<Lanelet>> findUpcomingIncomingLanelets(const std::shared_ptr<Lanelet>& lanelet);

    bool isLeftOf(int timeStep, const std::shared_ptr<Obstacle> &obsK, const std::shared_ptr<Obstacle> &obsP);

    std::vector<std::shared_ptr<Lanelet>> incomingLaneletsLeftOfLanelet(const std::shared_ptr<Lanelet> &lanelet);

    bool samePriority(int timeStep, const std::shared_ptr<Obstacle> &obsK, const std::shared_ptr<Obstacle> &obsP, TurningDirections dirK, TurningDirections dirP);

    bool upcomingTrafficLights(int timeStep, const std::shared_ptr<Obstacle> &obs);

    bool inConflictArea(int timeStep, const std::shared_ptr<Obstacle> &obsK, const std::shared_ptr<Obstacle> &obsP);

    static double rearPosition(int timeStep, const std::shared_ptr<Lane> &referenceLane, const std::shared_ptr<Obstacle> &obs);

    static double frontPosition(int timeStep, const std::shared_ptr<Lane> &referenceLane, const std::shared_ptr<Obstacle> &obs);

    bool onOncomingOf(int timeStep, const std::shared_ptr<Obstacle> &obsK, const std::shared_ptr<Obstacle> &obsP);

    static bool sameLeftRightOutgoing(int timeStep, const std::shared_ptr<Obstacle> &obsK, const std::shared_ptr<Obstacle> &obsP);

private:
    std::shared_ptr<RoadNetwork> roadNetwork;
    SupportedTrafficSignCountry country;
};


#endif //ENV_MODEL_PREDICATES_H
