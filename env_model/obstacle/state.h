//
// Created by Sebastian Maierhofer on 01.11.20.
//

#ifndef ENV_MODEL_STATE_H
#define ENV_MODEL_STATE_H

#include "../auxiliaryDefs/structs.h"
#include "geometry/curvilinear_coordinate_system.h"

typedef geometry::CurvilinearCoordinateSystem CurvilinearCoordinateSystem;

/**
 * Class representing a state.
 */
class State {
public:
    /**
     * Default Constructor for a state without parameters. Everything state variable is set to
     * 0.0 and all validities are set to false.
     */
    State() = default;

    /**
     * Constructor initializing all variables.
     */
    State(int timeStep, double xPosition, double yPosition, double velocity, double acceleration, double orientation,
          double lonPosition, double latPosition);

    /**
     * Constructor initializing all variables except longitudinal and lateral position.
     */
    State(int timeStep, double xPosition, double yPosition, double velocity, double acceleration, double orientation);

    /**
     * Getter for x-position.
     * @return x-position of the state in the Cartesian space.
     */
    [[nodiscard]] double getXPosition() const;

    /**
     * Getter for x-position.
     * @return x-position in Cartesian space.
     */
    [[nodiscard]] double getYPosition() const;

    /**
     * Getter for velocity.
     * @return velocity.
     */
    [[nodiscard]] double getVelocity() const;

    /**
     * Getter for acceleration.
     * @return acceleration.
     */
    [[nodiscard]] double getAcceleration() const;

    /**
     * Getter for longitudinal position.
     * @return longitudinal position in Curvilinear domain.
     */
    [[nodiscard]] double getLonPosition() const;

    /**
     * Getter for lateral position.
     * @return lateral position in Curvilinear domain.
     */
    [[nodiscard]] double getLatPosition() const;

    /**
     * Getter for orientation.
     * @return orientation.
     */
    [[nodiscard]] double getOrientation() const;

    /**
     * Getter for time step.
     * @return time step.
     */
    [[nodiscard]] int getTimeStep() const;

    /**
     * Getter for list of valid states elements.
     * @return validity struct.
     */
    [[nodiscard]] const ValidStates &getValidStates() const;

    /**
     * Setter for x-position in Cartesian space.
     * @param x-position in Cartesian space.
     */
    void setXPosition(double xPosition);

    /**
     * Setter for y-position in Cartesian space.
     * @param y-position in Cartesian space.
     */
    void setYPosition(double yPosition);

    /**
     * Setter for velocity.
     * @param velocity.
     */
    void setVelocity(double velocity);

    /**
     * Setter for acceleration.
     * @param acceleration.
     */
    void setAcceleration(double acceleration);

    /**
     * Setter for longitudinal position in Curvilinear domain.
     * @param longitudinal position in Curvilinear domain.
     */
    void setLonPosition(double lonPosition);

    /**
     * Setter for lateral position in Curvilinear domain.
     * @param lateral position in Curvilinear domain.
     */
    void setLatPosition(double latPosition);

    /**
     * Setter for orientation.
     * @param orientation.
     */
    void setOrientation(double orientation);

    /**
     * Setter for time step.
     * @param time step.
     */
    void setTimeStep(int timeStep);

    /**
    * Converts the x- and y-coordinate into the Curvilinear domain.
    *
    * @param ccs Curvilinear coordinate system in which projection should happen.
    */
    void convertPointToCurvilinear(const CurvilinearCoordinateSystem& ccs);

private:
        double xPosition{0.0};                                          //**< x-coordinate in Cartesian space [m] */
        double yPosition{0.0};                                          //**< y-coordinate in Cartesian space */
        double velocity{0.0};                                           //**< velocity [m/s] */
        double acceleration{0.0};                                       //**< acceleration [m/s^2] */
        double lonPosition{0.0};                                        //**< longitudinal position [m] */
        double latPosition{0.0};                                        //**< lateral position [m] */
        double orientation{0.0};                                        //**< orientation [rad] */
        ValidStates validStates{false, false,         //**< set of states which are already set and therefore are valid [rad] */
                                false,false,
                                false,false,
                                false};
        int timeStep{0};                                                //**< time step of the state variables */
};


#endif //ENV_MODEL_STATE_H