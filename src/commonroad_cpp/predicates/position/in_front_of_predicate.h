//
// Created by Sebastian Maierhofer on 07.04.21.
//

#ifndef ENV_MODEL_IN_FRONT_OF_PREDICATE_H
#define ENV_MODEL_IN_FRONT_OF_PREDICATE_H

#include "commonroad_cpp/predicates/predicate.h"

/**
 * Evaluates whether the kth vehicle is in front of the pth vehicle.
 */
class InFrontOfPredicate : public Predicate {
  public:
    /**
     * Boolean evaluation of predicate using parameter values.
     *
     * @param lonPositionK Longitudinal position of the kth obstacle [m].
     * @param lonPositionP Longitudinal position of the pth obstacle [m].
     * @param lengthK Length of the kth obstacle [m].
     * @param lengthP Length of the pth obstacle [m].
     * @return Boolean indicating satisfaction of the predicate.
     */
    static bool booleanEvaluation(double lonPositionK, double lonPositionP, double lengthK, double lengthP);

    /**
     * Boolean evaluation of predicate using objects.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle.
     * @return Constraints defined by the predicate.
     */
    bool booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                           const std::shared_ptr<Obstacle> &obstacleK,
                           const std::shared_ptr<Obstacle> &obstacleP) override;

    /**
     * Constraint evaluation of predicate using objects.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle.
     * @return Constraints defined by the predicate.
     */
    Constraint constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                    const std::shared_ptr<Obstacle> &obstacleK,
                                    const std::shared_ptr<Obstacle> &obstacleP) override;

    /**
     * Constraint evaluation of predicate using parameter values.
     *
     * @param lonPositionP Longitudinal position of the pth obstacle [m].
     * @param lengthK Length of the kth obstacle [m].
     * @param lengthP Length of the pth obstacle [m].
     * @return Constraints defined by the predicate.
     */
    static Constraint constraintEvaluation(double lonPositionP, double lengthK, double lengthP);

    /**
     * Robustness evaluation of predicate using objects.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @return Real value indicating robustness of the predicate.
     */
    double robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                            const std::shared_ptr<Obstacle> &obstacleK,
                            const std::shared_ptr<Obstacle> &obstacleP) override;

    /**
     * Robustness evaluation of predicate using parameter values.
     *
     * @param lonPositionK Longitudinal position of the kth obstacle [m].
     * @param lonPositionP Longitudinal position of the pth obstacle [m].
     * @param lengthK Length of the kth obstacle [m].
     * @param lengthP Length of the pth obstacle [m].
     * @return Real value indicating robustness of the predicate.
     */
    static double robustEvaluation(double lonPositionK, double lonPositionP, double lengthK, double lengthP);
};

#endif // ENV_MODEL_IN_FRONT_OF_PREDICATE_H
