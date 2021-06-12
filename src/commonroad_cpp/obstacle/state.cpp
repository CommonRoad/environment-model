//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "state.h"

State::State(size_t timeStep, double xPosition, double yPosition, double velocity, double acceleration,
             double globalOrientation, double curvilinearOrientation, double lonPosition, double latPosition)
    : xPosition(xPosition), yPosition(yPosition), velocity(velocity), acceleration(acceleration),
      lonPosition(lonPosition), latPosition(latPosition), globalOrientation(globalOrientation),
      curvilinearOrientation(curvilinearOrientation),
      validStates(ValidStates{true, true, true, true, true, true, true, true}), timeStep(timeStep) {}

State::State(size_t timeStep, double xPosition, double yPosition, double velocity, double acceleration,
             double orientation)
    : xPosition(xPosition), yPosition(yPosition), velocity(velocity), acceleration(acceleration),
      globalOrientation(orientation), validStates(ValidStates{true, true, true, true, false, false, true, false}),
      timeStep(timeStep) {}

double State::getXPosition() const { return xPosition; }

void State::setXPosition(double x) {
    validStates.xPosition = true;
    xPosition = x;
}

double State::getYPosition() const { return yPosition; }

void State::setYPosition(double y) {
    validStates.yPosition = true;
    yPosition = y;
}

double State::getVelocity() const { return velocity; }

void State::setVelocity(double vel) {
    validStates.velocity = true;
    State::velocity = vel;
}

double State::getAcceleration() const {
    if (!validStates.acceleration)
        throw std::runtime_error("State::getAcceleration acceleration not initialized");
    return acceleration;
}

void State::setAcceleration(double acc) {
    validStates.acceleration = true;
    acceleration = acc;
}

double State::getLonPosition() const {
    if (!validStates.lonPosition)
        throw std::runtime_error("State::getLonPosition longitudinal position not initialized");
    return lonPosition;
}

void State::setLonPosition(double s) {
    validStates.lonPosition = true;
    lonPosition = s;
}

double State::getLatPosition() const {
    if (!validStates.latPosition)
        throw std::runtime_error("State::getLatPosition lateral position not initialized");
    return latPosition;
}

void State::setLatPosition(double d) {
    validStates.latPosition = true;
    latPosition = d;
}

double State::getGlobalOrientation() const { return globalOrientation; }

void State::setGlobalOrientation(double theta) {
    validStates.globalOrientation = true;
    globalOrientation = theta;
}

double State::getCurvilinearOrientation() const {
    if (!validStates.curvilinearOrientation)
        throw std::runtime_error("State::getCurvilinearOrientation curvilinear orientation not initialized");
    return curvilinearOrientation;
}

void State::setCurvilinearOrientation(double theta) {
    validStates.curvilinearOrientation = true;
    curvilinearOrientation = theta;
}

size_t State::getTimeStep() const { return timeStep; }

void State::setTimeStep(size_t time) { timeStep = time; }

const ValidStates &State::getValidStates() const { return validStates; }
