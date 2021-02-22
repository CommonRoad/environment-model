//
// Created by Sebastian Maierhofer on 19.02.21.
//

#ifndef ENV_MODEL_SAFE_DISTANCE_PREDICATE_H
#define ENV_MODEL_SAFE_DISTANCE_PREDICATE_H

#include "predicate.h"

/**
 * Predicate for the safe distance between two vehicles.
 */
class SafeDistancePredicate : public Predicate {
public:
    /**
     * Safe distance between the kth and the pth obstacle.
     *
     * @param velocityK Velocity of the kth obstacle [m/s].
     * @param velocityP Velocity of the pth obstacle [m/s].
     * @param minAccelerationK Minimum (longitudinal) acceleration of the kth obstacle [m/s^2].
     * @param minAccelerationP Minimum (longitudinal) acceleration of the pth obstacle [m/s^2].
     * @param tReact Reaction time of the kth obstacle [s].
     * @return Safe distance [m].
     */
    static double
    computeSafeDistance(double velocityK, double velocityP, double minAccelerationK, double minAccelerationP,
                        double tReact);

    /**
     * Boolean evaluation of predicate.
     *
     * @param timeStep Time step of interest.
     * @param roadNetwork RoadNetwork object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @param obstacles Pointers to all obstacles. This is an optional parameter.
     * @return Boolean indicating satisfaction of the predicate.
     */
    bool booleanEvaluation(int timeStep,
                           const std::shared_ptr<RoadNetwork> &roadNetwork,
                           const std::shared_ptr<Obstacle> &obstacleK,
                           const std::shared_ptr<Obstacle> &obstacleP,
                           const std::vector<std::shared_ptr<Obstacle>> &obstacles) override;

    /**
     * Boolean evaluation of predicate.
     *
     * @param timeStep Time step of interest.
     * @param roadNetwork RoadNetwork object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @return Boolean indicating satisfaction of the predicate.
     */
    bool booleanEvaluation(int timeStep,
                           const std::shared_ptr<RoadNetwork> &roadNetwork,
                           const std::shared_ptr<Obstacle> &obstacleK,
                           const std::shared_ptr<Obstacle> &obstacleP);

    /**
     *
     * @param lonPosK Longitudinal position in the curvilinear coordinate system of the kth vehicle [m].
     * @param lonPosP Longitudinal position in the curvilinear coordinate system of the pth vehicle [m].
     * @param velocityK Velocity of the kth vehicle [m/s].
     * @param velocityP Velocity of the pth vehicle [m/s].
     * @param minAccelerationK Minimum (longitudinal) acceleration of the kth obstacle [m/s^2].
     * @param minAccelerationP Minimum (longitudinal) acceleration of the pth obstacle [m/s^2].
     * @param tReact Reaction time of the kth obstacle [s].
     * @return Boolean indicating satisfaction of the predicate.
     */
    bool
    booleanEvaluation(double lonPosK, double lonPosP, double velocityK, double velocityP, double minAccelerationK,
                      double minAccelerationP, double tReact);

    /**
     * Constraint evaluation of predicate.
     *
     * @param timeStep Time step of interest.
     * @param roadNetwork RoadNetwork object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @param obstacles Pointers to all obstacles. This is an optional parameter.
     * @return Constraints defined by the predicate.
     */
    Constraint constraintEvaluation(int timeStep,
                                    const std::shared_ptr<RoadNetwork> &roadNetwork,
                                    const std::shared_ptr<Obstacle> &obstacleK,
                                    const std::shared_ptr<Obstacle> &obstacleP,
                                    const std::vector<std::shared_ptr<Obstacle>> &obstacles) override;

    /**
     * Constraint evaluation of predicate.
     *
     * @param timeStep Time step of interest.
     * @param roadNetwork RoadNetwork object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @return Constraints defined by the predicate.
     */
    Constraint constraintEvaluation(int timeStep,
                                    const std::shared_ptr<RoadNetwork> &roadNetwork,
                                    const std::shared_ptr<Obstacle> &obstacleK,
                                    const std::shared_ptr<Obstacle> &obstacleP);

    /**
     *
     * @param lonPosK Longitudinal position in the curvilinear coordinate system of the kth vehicle [m].
     * @param lonPosP Longitudinal position in the curvilinear coordinate system of the pth vehicle [m].
     * @param velocityK Velocity of the kth vehicle [m/s].
     * @param velocityP Velocity of the pth vehicle [m/s].
     * @param minAccelerationK Minimum (longitudinal) acceleration of the kth obstacle [m/s^2].
     * @param minAccelerationP Minimum (longitudinal) acceleration of the pth obstacle [m/s^2].
     * @param tReact Reaction time of the kth obstacle [s].
     * @return Constraints defined by the predicate.
     */
    Constraint
    constraintEvaluation(double velocityK, double velocityP, double minAccelerationK, double minAccelerationP,
                         double tReact);

    /**
     * Robustness evaluation of predicate.
     *
     * @param timeStep Time step of interest.
     * @param roadNetwork RoadNetwork object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @param obstacles Pointers to all obstacles. This is an optional parameter.
     * @return Real value indicating robustness of the predicate.
     */
    double robustEvaluation(int timeStep,
                            const std::shared_ptr<RoadNetwork> &roadNetwork,
                            const std::shared_ptr<Obstacle> &obstacleK,
                            const std::shared_ptr<Obstacle> &obstacleP,
                            const std::vector<std::shared_ptr<Obstacle>> &obstacles) override;

    /**
     * Robustness evaluation of predicate.
     *
     * @param timeStep Time step of interest.
     * @param roadNetwork RoadNetwork object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @return Real value indicating robustness of the predicate.
     */
    double robustEvaluation(int timeStep,
                            const std::shared_ptr<RoadNetwork> &roadNetwork,
                            const std::shared_ptr<Obstacle> &obstacleK,
                            const std::shared_ptr<Obstacle> &obstacleP);

    /**
     *
     * @param velocityK Velocity of the kth vehicle [m/s].
     * @param velocityP Velocity of the pth vehicle [m/s].
     * @param minAccelerationK Minimum (longitudinal) acceleration of the kth obstacle [m/s^2].
     * @param minAccelerationP Minimum (longitudinal) acceleration of the pth obstacle [m/s^2].
     * @param tReact Reaction time of the kth obstacle [s].
     * @return Real value indicating robustness of the predicate.
     */
    double
    robustEvaluation(double lonPosK, double lonPosP, double velocityK, double velocityP, double minAccelerationK,
                     double minAccelerationP, double tReact);
};


#endif //ENV_MODEL_SAFE_DISTANCE_PREDICATE_H
