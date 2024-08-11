#pragma once

#include <memory>
#include <mutex>

#include "commonroad_cpp/auxiliaryDefs/structs.h"
#include "commonroad_cpp/auxiliaryDefs/timer.h"

#include "predicate_parameter_collection.h"

class Obstacle;
class World;

struct OptionalPredicateParameters {
    OptionalPredicateParameters() = default;
    OptionalPredicateParameters(std::vector<TrafficSignTypes> signType, std::vector<LaneletType> laneletType,
                                std::vector<TurningDirection> turningDirection,
                                std::vector<TrafficLightState> trafficLightState);
    OptionalPredicateParameters(std::vector<TrafficSignTypes> signType);
    OptionalPredicateParameters(std::vector<LaneletType> laneletType);
    OptionalPredicateParameters(std::vector<TurningDirection> turningDirection);
    OptionalPredicateParameters(std::vector<TrafficLightState> trafficLightState);
    OptionalPredicateParameters(double distance);
    std::vector<TrafficSignTypes> signType;
    std::vector<LaneletType> laneletType;
    std::vector<TurningDirection> turningDirection;
    std::vector<TrafficLightState> trafficLightState;
    ObstacleType obstacleType;
    double minSafetyDistance;
    double acceleration;       // e.g. relative acceleration value
    double velocity_threshold; // e.g. congestion velocity
};

/**
 * Interface for a predicate.
 */
class CommonRoadPredicate {
  public:
    /**
     * Default constructor for predicate class without parameter.
     *
     * @param vehicleDependent Boolean indicating whether predicate depends only on one vehicle
     */
    CommonRoadPredicate(bool vehicleDependent);

    virtual ~CommonRoadPredicate();

    CommonRoadPredicate(const CommonRoadPredicate &) = delete;

    /**
     * Virtual function for the boolean evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @param additionalFunctionParameters Additional parameters.
     * @return Boolean indicating satisfaction of the predicate.
     */
    virtual bool
    booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
                      const std::shared_ptr<Obstacle> &obstacleP,
                      const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters = {}) = 0;

    // FIXME: Make this internal if possible (friend function?)
    /**
     * Variant of booleanEvaluation without optional parameter.
     * FOR INTERNAL USE ONLY
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @return Boolean indicating satisfaction of the predicate.
     */
    bool simpleBooleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                 const std::shared_ptr<Obstacle> &obstacleK,
                                 const std::shared_ptr<Obstacle> &obstacleP);

    /**
     * Virtual function for the robustness evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @param additionalFunctionParameters Additional parameters.
     * @return Real value indicating robustness of the predicate.
     */
    virtual double
    robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
                     const std::shared_ptr<Obstacle> &obstacleP,
                     const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters = {}) = 0;

    /**
     * Virtual function for the constraint evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @return Constraints defined by the predicate.
     */
    virtual Constraint
    constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                         const std::shared_ptr<Obstacle> &obstacleK, const std::shared_ptr<Obstacle> &obstacleP,
                         const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters = {}) = 0;

    /**
     * Function for the statistical evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     */
    bool statisticBooleanEvaluation(
        size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
        const std::shared_ptr<Timer> &evaluationTimer, const std::shared_ptr<PredicateStatistics> &statistics,
        const std::shared_ptr<Obstacle> &obstacleP = {},
        const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters = {});

    /**
     * Getter for parameters.
     *
     * @return Struct containing all parameters.
     */
    [[nodiscard]] PredicateParameters &getParameters();

    /**
     * Setter for parameters.
     *
     * @param parameters Struct containing all parameters.
     */
    void setParameters(const PredicateParameters &parameters);

    /**
     * Returns whether predicate is vehicle dependent.
     *
     * @return Boolean indicating whether predicate is vehicle dependent.
     */
    [[nodiscard]] bool isVehicleDependent() const;

  protected:
    PredicateParameters parameters; //**< Struct containing parameters of all predicates. */
    const bool vehicleDependent; //**< Boolean indicating whether predicate depends on one specific obstacle or two. */
};

extern std::map<std::string, std::shared_ptr<CommonRoadPredicate>> predicates; //**< List of all predicates **/
