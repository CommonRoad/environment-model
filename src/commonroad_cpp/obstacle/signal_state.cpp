//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "commonroad_cpp/obstacle/signal_state.h"
#include <stdexcept>

size_t SignalState::getTimeStep() const { return timeStep; }

SignalState::SignalState(size_t timeStep, bool horn, bool indicatorLeft, bool indicatorRight, bool brakingLights,
                         bool hazardWarningLights, bool flashingBlueLights)
    : horn(horn), indicatorLeft(indicatorLeft), indicatorRight(indicatorRight), brakingLights(brakingLights),
      hazardWarningLights(hazardWarningLights), flashingBlueLights(flashingBlueLights), timeStep(timeStep) {}

bool SignalState::isHorn() const { return horn; }

bool SignalState::isIndicatorLeft() const { return indicatorLeft; }

bool SignalState::isIndicatorRight() const { return indicatorRight; }

bool SignalState::isBrakingLights() const { return brakingLights; }

bool SignalState::isHazardWarningLights() const { return hazardWarningLights; }

bool SignalState::isFlashingBlueLights() const { return flashingBlueLights; }

void SignalState::setHorn(bool hornStatus) { horn = hornStatus; }

void SignalState::setIndicatorLeft(bool inl) { indicatorLeft = inl; }

void SignalState::setIndicatorRight(bool inr) { indicatorRight = inr; }

void SignalState::setBrakingLights(bool bls) { brakingLights = bls; }

void SignalState::setHazardWarningLights(bool hwl) { hazardWarningLights = hwl; }

void SignalState::setFlashingBlueLights(bool fbl) { flashingBlueLights = fbl; }

void SignalState::setTimeStep(size_t tsp) { timeStep = tsp; }
