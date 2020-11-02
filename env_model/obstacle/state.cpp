//
// Created by sebastian on 01.11.20.
//

#include "state.h"
#include "stdlib.h"

double State::getXPosition() const {
    return xPosition;
}

void State::setXPosition(double x) {
    xPosition = x;
}

double State::getYPosition() const {
    return yPosition;
}

void State::setYPosition(double y) {
    yPosition = y;
}

double State::getVelocity() const {
    return velocity;
}

void State::setVelocity(double vel) {
    State::velocity = vel;
}

double State::getAcceleration() const {
    return acceleration;
}

void State::setAcceleration(double acc) {
    acceleration = acc;
}

double State::getLonPosition() const {
    return lonPosition;
}

void State::setLonPosition(double s) {
    lonPosition = s;
}

double State::getLatPosition() const {
    return latPosition;
}

void State::setLatPosition(double d) {
    latPosition = d;
}

double State::getOrientation() const {
    return orientation;
}

void State::setOrientation(double theta) {
    orientation = theta;
}

int State::getTimeStep() const {
    return timeStep;
}

void State::setTimeStep(int time) {
    timeStep = time;
}
