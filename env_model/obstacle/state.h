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
    void setValidStates(const ValidStates &validStates);
    void convertPointToCurvilinear(const CurvilinearCoordinateSystem& ccs);

private:
        double xPosition{0.0};
        double yPosition{0.0};
        double velocity{0.0};
        double acceleration{0.0};
        double lonPosition{0.0};
        double latPosition{0.0};
        double orientation{0.0};
        ValidStates validStates{false, false, false, false, false,
                                false, false};
        int timeStep{0};
};


#endif //ENV_MODEL_STATE_H