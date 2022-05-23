#pragma once

#include <cassert>
#include <optional>

/**
 * SensorParameters includes information regarding obstacle sensors
 *
 * Note: The following assumptions about the quantites are enforced in the constructors:
 *   - \f$reactionTime \geq 0.0\f$
 *
 */
class SensorParameters {
    double fieldOfViewRear{250.0};  //**< length of field of view provided by rear sensors */
    double fieldOfViewFront{250.0}; //**< length of field of view provided by front sensors */

    /** reaction time of obstacle in [s] */
    std::optional<double> reactionTime;

  public:
    SensorParameters() = delete;

    /**
     * Complete constructor for SensorParameters.
     *
     * @param fieldOfViewRear length of field of view provided by rear sensors
     * @param fieldOfViewFront length of field of view provided by front sensors
     * @param reactionTime reaction time of obstacle in [s]
     */
    SensorParameters(double fieldOfViewRear, double fieldOfViewFront, std::optional<double> reactionTime = std::nullopt)
        : fieldOfViewRear{fieldOfViewRear}, fieldOfViewFront{fieldOfViewFront}, reactionTime{reactionTime} {
        // TODO These assertions might make more sense as a std::domain_error exception
        assert(reactionTime.value_or(0.0) >= 0.0);
    }

    /**
     * Simplified constructor for SensorParameters.
     *
     * @param reactionTime reaction time of obstacle in [s]
     */
    SensorParameters(std::optional<double> reactionTime = std::nullopt) : reactionTime{reactionTime} {
        // TODO These assertions might make more sense as a std::domain_error exception
        assert(reactionTime.value_or(0.0) >= 0.0);
    }

    /**
     * Getter for rear field of view.
     *
     * @return Rear field of view.
     */
    [[nodiscard]] double getFieldOfViewRear() const noexcept { return fieldOfViewRear; }

    /**
     * Getter for front field of view.
     *
     * @return Front field of view.
     */
    [[nodiscard]] double getFieldOfViewFront() const noexcept { return fieldOfViewFront; }

    /**
     * Getter for reaction time.
     *
     * @return Reaction time [s].
     */
    [[nodiscard]] std::optional<double> getReactionTime() const noexcept { return reactionTime; }

    /**
     * Default kinematic parameters for vehicles:
     * fieldOfViewRear = fieldOfViewFront = 250,
     * reaction time = 0.3 s.
     *
     *
     * @return Default vehicle kinematic parameters.
     */
    static SensorParameters dynamicDefaults() { return SensorParameters{250.0, 250.0, 0.3}; }

    /**
     * Default kinematic parameters for static obstacles:
     * vMax = 0.0 m/s, aMax = 0.0 m/s^2.
     *
     * @return Default static obstacle kinematic parameters.
     */
    static SensorParameters staticDefaults() { return SensorParameters{0.0, 0.0, std::nullopt}; }
};
