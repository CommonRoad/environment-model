#pragma once

#include <cassert>
#include <optional>

/*
 * TODO: Possible candidates for inclusion

 * orientation/acceleration/velocity error
 * vehicle-specific: wheelbase, max steering angle
 * comfortable acceleration limit
 */

/**
 * KinematicParameters includes all the static information required in order to predict
 * the motion of an obstacle, i.e., acceleration and velocity bounds but no dynamic
 * information such as current velocity or acceleration.
 *
 * KinematicParameters + State + Global Parameters = Information required for Occupancy Prediction
 *
 * Note: The following assumptions about the quantites are enforced in the constructors:
 *   - \f$vMax \geq 0.0\f$
 *   - \f$aMax \geq 0.0\f$
 *   - \f$aMaxLong \geq 0.0\f$
 *   - \f$aMinLong \leq 0.0\f$
 *   - \f$aBraking \leq 0.0\f$
 *   - \f$aMaxLong \leq aMax\f$
 *   - \f$|aBraking| \leq |aMinLong|\f$
 *   - \f$reactionTime \geq 0.0\f$
 *
 */
class KinematicParameters {
    /** maximum velocity of obstacle in m/s */
    double vMax;
    /** maximum absolute acceleration of obstacle in [m/s^2] */
    double aMax;
    /** maximal longitudinal acceleration of obstacle in [m/s^2] */
    double aMaxLong;
    /** minimal longitudinal acceleration of obstacle in [m/s^2] */
    double aMinLong;
    /** minimal (longitudinal) braking acceleration of obstacle in [m/s^2] */
    double aBraking;
    /** reaction time of obstacle in [s] */
    std::optional<double> reactionTime;

  public:
    KinematicParameters() = delete;

    /**
     * Complete constructor for KinematicParameters.
     *
     * @param vMax maximum velocity of obstacle in [m/s]
     * @param aMax maximum absolute acceleration of obstacle in [m/s^2]
     * @param aMaxLong maximal longitudinal acceleration of obstacle in [m/s^2]
     * @param aMinLong minimal longitudinal acceleration of obstacle in [m/s^2]
     * @param aBraking minimal (longitudinal) braking acceleration of obstacle in [m/s^2]
     * @param reactionTime reaction time of obstacle in [s]
     */
    KinematicParameters(double vMax, double aMax, double aMaxLong, double aMinLong, double aBraking,
                        std::optional<double> reactionTime = std::nullopt)
        : vMax{vMax}, aMax{aMax}, aMaxLong{aMaxLong}, aMinLong{aMinLong}, aBraking{aBraking}, reactionTime{
                                                                                                  reactionTime} {
        // TODO These assertions might make more sense as a std::domain_error exception
        assert(vMax >= 0.0);
        assert(aMax >= 0.0);
        assert(aMaxLong >= 0.0);
        assert(aMinLong <= 0.0);
        assert(aBraking <= 0.0);
        assert(aMaxLong <= aMax);
        assert(std::abs(aBraking) <= std::abs(aMinLong));
        assert(reactionTime.value_or(0.0) >= 0.0);

        // Currently disabled, based on current assumptions
        // assert(std::abs(aBraking) < aMax);
        // assert(std::abs(aMinLong) < aMax);
    }

    /**
     * Simplified constructor for KinematicParameters,
     * setting all acceleration limits based on aMax.
     *
     * @param vMax maximum velocity of obstacle in [m/s]
     * @param aMax maximum absolute acceleration of obstacle in [m/s^2]
     */
    KinematicParameters(double vMax, double aMax, std::optional<double> reactionTime = std::nullopt)
        : KinematicParameters{vMax, aMax, aMax, -aMax, -aMax, reactionTime} {}

    /**
     * Getter for maximum velocity the vehicle can drive.
     *
     * @return Maximum velocity [m/s].
     */
    [[nodiscard]] double getVmax() const noexcept { return vMax; }

    /**
     * Getter for maximum acceleration.
     *
     * @return Maximum acceleration [m/s^2].
     */
    [[nodiscard]] double getAmax() const noexcept { return aMax; }

    /**
     * Getter for maximum acceleration in longitudinal direction.
     *
     * @return Maximum acceleration in longitudinal direction [m/s^2].
     */
    [[nodiscard]] double getAmaxLong() const noexcept { return aMaxLong; }

    /**
     * Getter for minimum acceleration in longitudinal direction.
     *
     * @return Minimum acceleration in longitudinal direction [m/s^2].
     */
    [[nodiscard]] double getAminLong() const noexcept { return aMinLong; }

    /**
     * Getter for minimum braking acceleration in longitudinal direction.
     *
     * @return Minimum acceleration in longitudinal direction [m/s^2].
     */
    [[nodiscard]] double getAbraking() const noexcept { return aBraking; }

    /**
     * Getter for reaction time.
     *
     * @return Reaction time [s].
     */
    [[nodiscard]] std::optional<double> getReactionTime() const noexcept { return reactionTime; }

    /**
     * Default kinematic parameters for vehicles:
     * vMax = 50.0 m/s, aMax = 3.0 m/s^2, aMaxLong = 3 m/s^2, aMinLong = -10.0 m/s^2, aBraking = -5.0 m/s^2,
     * reaction time = 0.3 s.
     *
     *
     * @return Default vehicle kinematic parameters.
     */
    static KinematicParameters vehicleDefaults() { return KinematicParameters{50.0, 3.0, 3.0, -10.0, -5.0, 0.3}; }

    /**
     * Default kinematic parameters for pedestrians:
     * vMax = 2.0 m/s, aMax = 0.6 m/s^2, other limits based on aMax,
     * reaction time = 0.3 s.
     *
     * @return Default pedestrain kinematic parameters.
     */
    static KinematicParameters pedestrianDefaults() { return KinematicParameters{2.0, 0.6, 0.3}; }

    /**
     * Default kinematic parameters for static obstacles:
     * vMax = 0.0 m/s, aMax = 0.0 m/s^2.
     *
     * @return Default static obstacle kinematic parameters.
     */
    static KinematicParameters staticDefaults() { return KinematicParameters{0.0, 0.0, std::nullopt}; }
};
