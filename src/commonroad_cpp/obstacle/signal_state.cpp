#include "commonroad_cpp/obstacle/signal_state.h"
#include <algorithm>
#include <spdlog/spdlog.h>
#include <sstream>

size_t SignalState::getTimeStep() const { return timeStep; }

SignalState::SignalState(const size_t timeStep, const bool horn, const bool indicatorLeft, const bool indicatorRight,
                         const bool brakingLights, const bool hazardWarningLights, const bool flashingBlueLights)
    : horn(horn), indicatorLeft(indicatorLeft), indicatorRight(indicatorRight), brakingLights(brakingLights),
      hazardWarningLights(hazardWarningLights), flashingBlueLights(flashingBlueLights), timeStep(timeStep) {}

bool SignalState::isHorn() const { return horn; }

bool SignalState::isIndicatorLeft() const { return indicatorLeft; }

bool SignalState::isIndicatorRight() const { return indicatorRight; }

bool SignalState::isBrakingLights() const { return brakingLights; }

bool SignalState::isHazardWarningLights() const { return hazardWarningLights; }

bool SignalState::isFlashingBlueLights() const { return flashingBlueLights; }

void SignalState::setHorn(const bool hornStatus) { horn = hornStatus; }

void SignalState::setIndicatorLeft(const bool inl) { indicatorLeft = inl; }

void SignalState::setIndicatorRight(const bool inr) { indicatorRight = inr; }

void SignalState::setBrakingLights(const bool bls) { brakingLights = bls; }

void SignalState::setHazardWarningLights(const bool hwl) { hazardWarningLights = hwl; }

void SignalState::setFlashingBlueLights(const bool fbl) { flashingBlueLights = fbl; }

void SignalState::setTimeStep(const size_t tsp) { timeStep = tsp; }

bool SignalState::isSignalSet(const std::string &signalName) const {
    auto sigNameTmp{signalName};
    std::transform(sigNameTmp.begin(), sigNameTmp.end(), sigNameTmp.begin(), tolower);
    if (sigNameTmp == "horn")
        return isHorn();
    if (sigNameTmp == "indicatorleft")
        return isIndicatorLeft();
    if (sigNameTmp == "indicatorright")
        return isIndicatorRight();
    if (sigNameTmp == "brakinglights")
        return isBrakingLights();
    if (sigNameTmp == "hazardwarninglights")
        return isHazardWarningLights();
    if (sigNameTmp == "flashingbluelights")
        return isFlashingBlueLights();
    spdlog::error("SignalState::isSignalSet: Unknown signal name!");
    return false;
}

std::string SignalState::to_string() const {
    std::ostringstream oss;
    oss << "SignalState[timeStep=" << timeStep << ", horn=" << horn << ", indicatorLeft=" << indicatorLeft
        << ", indicatorRight=" << indicatorRight << ", brakingLights=" << brakingLights
        << ", hazardWarningLights=" << hazardWarningLights << ", flashingBlueLights=" << flashingBlueLights << "]";
    return oss.str();
}
