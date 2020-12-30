//
// Created by Sebastian Maierhofer on 01.11.20.
//

#include "state.h"

State::State(int timeStep,
             double xPosition,
             double yPosition,
             double velocity,
             double acceleration,
             double orientation,
             double lonPosition,
             double latPosition) :
        timeStep(timeStep),
        xPosition(xPosition),
        yPosition(yPosition),
        velocity(velocity),
        acceleration(acceleration),
        lonPosition(lonPosition),
        latPosition(latPosition),
        orientation(orientation),
        validStates(ValidStates{true, true, true, true,
                                true, true, true}) {}

State::State(int timeStep,
             double xPosition,
             double yPosition,
             double velocity,
             double acceleration,
             double orientation) :
        timeStep(timeStep),
        xPosition(xPosition),
        yPosition(yPosition),
        velocity(velocity),
        acceleration(acceleration),
        orientation(orientation),
        validStates(ValidStates{true, true, true, true,
                                false, false, true}) {}

double State::getXPosition() const {
    return xPosition;
}

void State::setXPosition(double x) {
    validStates.xPosition = true;
    xPosition = x;
}

double State::getYPosition() const {
    return yPosition;
}

void State::setYPosition(double y) {
    validStates.yPosition = true;
    yPosition = y;
}

double State::getVelocity() const {
    return velocity;
}

void State::setVelocity(double vel) {
    validStates.velocity = true;
    State::velocity = vel;
}

double State::getAcceleration() const {
    return acceleration;
}

void State::setAcceleration(double acc) {
    validStates.acceleration = true;
    acceleration = acc;
}

double State::getLonPosition() const {
    return lonPosition;
}

void State::setLonPosition(double s) {
    validStates.lonPosition = true;
    lonPosition = s;
}

double State::getLatPosition() const {
    return latPosition;
}

void State::setLatPosition(double d) {
    validStates.latPosition = true;
    latPosition = d;
}

double State::getOrientation() const {
    return orientation;
}

void State::setOrientation(double theta) {
    validStates.orientation = true;
    orientation = theta;
}

int State::getTimeStep() const {
    return timeStep;
}

void State::setTimeStep(int time) {
    timeStep = time;
}

const ValidStates &State::getValidStates() const {
    return validStates;
}

void State::convertPointToCurvilinear(const CurvilinearCoordinateSystem &ccs) {
    Eigen::Vector2d convertedPoint = ccs.convertToCurvilinearCoords(getXPosition(), getYPosition());
    setLonPosition(convertedPoint.x());
    setLatPosition(convertedPoint.y());
}


