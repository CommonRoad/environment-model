#pragma once

#include <memory>

#include "commonroad_cpp/auxiliaryDefs/structs.h"
#include "commonroad_cpp/auxiliaryDefs/timer.h"

#include "predicate_parameter_collection.h"

class Obstacle;
class World;

/**
 * Interface for a predicate.
 *
 * For a set-based prediction, a predicate is evaluated on the occupancy, which covers many behaviors of an obstacle. A
 * must-version holds only if the predicate holds for all covered behaviors, a may-version if it holds for at least one.
 * Which version is sound depends on how the predicate is used, e.g., on its polarity in a rule, so the environment
 * model provides the versions as separate predicates with the suffix _must or _may (e.g., in_front_of_may), and the
 * caller selects one by name.
 *
 * TODO: Workaround. Only some predicates provide both versions; the set-based evaluation of all other predicates
 * (setBased = true) is a single version that is not classified as must or may yet. Classify them and add the missing
 * versions where a rule needs them. If the caller selects the versions automatically from the polarity of each
 * occurrence (see the TODO in commonroad-monitor, TrafficRule::initRule), consider replacing the separate predicates
 * by a mode parameter of the evaluation functions.
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
     * @param setBased Boolean indicating whether set-based evaluation should be used.
     * @return Boolean indicating satisfaction of the predicate.
     */
    virtual bool booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                   const std::shared_ptr<Obstacle> &obstacleK,
                                   const std::shared_ptr<Obstacle> &obstacleP,
                                   const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                                   bool setBased = false) = 0;

    // FIXME: Make this internal if possible (friend function?)
    /**
     * Variant of booleanEvaluation without optional parameter.
     * FOR INTERNAL USE ONLY
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @param additionalFunctionParameters Additional parameters.
     * @param setBased Boolean indicating whether set-based evaluation should be used.
     * @return Boolean indicating satisfaction of the predicate.
     */
    bool simpleBooleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                 const std::shared_ptr<Obstacle> &obstacleK, const std::shared_ptr<Obstacle> &obstacleP,
                                 const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                                 bool setBased = false);

    /**
     * Virtual function for the robustness evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @param additionalFunctionParameters Additional parameters.
     * @param setBased Boolean indicating whether set-based evaluation should be used.
     * @return Real value indicating robustness of the predicate.
     */
    virtual double robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                    const std::shared_ptr<Obstacle> &obstacleK,
                                    const std::shared_ptr<Obstacle> &obstacleP,
                                    const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                                    bool setBased = false) = 0;

    /**
     * Virtual function for the constraint evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @param additionalFunctionParameters Additional parameters.
     * @param setBased Boolean indicating whether set-based evaluation should be used.
     * @return Constraints defined by the predicate.
     */
    virtual Constraint constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                            const std::shared_ptr<Obstacle> &obstacleK,
                                            const std::shared_ptr<Obstacle> &obstacleP,
                                            const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                                            bool setBased = false) = 0;

    /**
     * Function for the statistical evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param evaluationTimer Timer which measures computation time.
     * @param statistics Statistic of predicates.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @param additionalFunctionParameters Additional parameters.
     * @param setBased Boolean indicating whether set-based evaluation should be used.
     * @return Boolean indicating satisfaction
     */
    bool statisticBooleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                    const std::shared_ptr<Obstacle> &obstacleK,
                                    const std::shared_ptr<Timer> &evaluationTimer,
                                    const std::shared_ptr<PredicateStatistics> &statistics,
                                    const std::shared_ptr<Obstacle> &obstacleP = {},
                                    const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                                    bool setBased = false);

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
    /**
     * Returns whether an obstacle is evaluated on its set-based prediction, i.e., on the occupancy at a future time
     * step.
     *
     * @param obstacle Obstacle of interest.
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based evaluation should be used.
     * @return Boolean indicating whether the occupancy of the obstacle is evaluated.
     */
    static bool evaluatesOccupancy(const std::shared_ptr<Obstacle> &obstacle, size_t timeStep, bool setBased);

    PredicateParameters parameters; //**< Struct containing parameters of all predicates. */
    const bool vehicleDependent; //**< Boolean indicating whether predicate depends on one specific obstacle or two. */
};

extern std::map<std::string, std::shared_ptr<CommonRoadPredicate>> predicates; //**< List of all predicates **/
