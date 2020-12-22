//
// Created by Sebastian Maierhofer on 01.11.20.
//

#ifndef ENV_MODEL_STATE_H
#define ENV_MODEL_STATE_H

#include "../auxiliaryDefs/structs.h"
#include "geometry/curvilinear_coordinate_system.h"

typedef geometry::CurvilinearCoordinateSystem CurvilinearCoordinateSystem;

class State {
public:
    State() = default;
    State(int timeStep, double xPosition, double yPosition, double velocity, double acceleration, double orientation,
          double lonPosition, double latPosition);
    State(int timeStep, double xPosition, double yPosition, double velocity, double acceleration, double orientation);

    [[nodiscard]] double getXPosition() const;
    [[nodiscard]] double getYPosition() const;
    [[nodiscard]] double getVelocity() const;
    [[nodiscard]] double getAcceleration() const;
    [[nodiscard]] double getLonPosition() const;
    [[nodiscard]] double getLatPosition() const;
    [[nodiscard]] double getOrientation() const;
    [[nodiscard]] int getTimeStep() const;
    [[nodiscard]] const ValidStates &getValidStates() const;

    void setXPosition(double xPosition);
    void setYPosition(double yPosition);
    void setVelocity(double velocity);
    void setAcceleration(double acceleration);
    void setLonPosition(double lonPosition);
    void setLatPosition(double latPosition);
    void setOrientation(double orientation);
    void setTimeStep(int timeStep);

    /**
    * Converts the x- and y-coordinate into the Curvilinear domain
    *
    * @param ccs Curvilinear coordinate system in which projection should happen
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