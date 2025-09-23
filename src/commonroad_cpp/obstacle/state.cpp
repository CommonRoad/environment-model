#include <commonroad_cpp/obstacle/state.h>
#include <sstream>
#include <stdexcept>

State::State(const size_t timeStep, const double xPosition, const double yPosition, const double velocity,
             const double acceleration, const double globalOrientation, const double curvilinearOrientation,
             const double lonPosition, const double latPosition)
    : xPosition(xPosition), yPosition(yPosition), velocity(velocity), acceleration(acceleration),
      lonPosition(lonPosition), latPosition(latPosition), globalOrientation(globalOrientation),
      curvilinearOrientation(curvilinearOrientation),
      validStates(ValidStates{true, true, true, true, true, true, true, true}), timeStep(timeStep) {}

State::State(const size_t timeStep, const double xPosition, const double yPosition, const double velocity,
             const double acceleration, const double orientation)
    : xPosition(xPosition), yPosition(yPosition), velocity(velocity), acceleration(acceleration),
      globalOrientation(orientation), validStates(ValidStates{true, true, true, true, false, false, true, false}),
      timeStep(timeStep) {}

double State::getXPosition() const { return xPosition; }

void State::setXPosition(const double xPos) {
    xPosition = xPos;
    validStates.xPosition = true;
}

double State::getYPosition() const { return yPosition; }

void State::setYPosition(const double yPos) {
    yPosition = yPos;
    validStates.yPosition = true;
}

double State::getVelocity() const { return velocity; }

void State::setVelocity(const double vel) {
    velocity = vel;
    validStates.velocity = true;
}

double State::getAcceleration() const {
    if (!validStates.acceleration)
        throw std::runtime_error("State::getAcceleration acceleration not initialized");
    return acceleration;
}

void State::setAcceleration(const double acc) {
    acceleration = acc;
    validStates.acceleration = true;
}

double State::getLonPosition() const {
    if (!validStates.lonPosition)
        throw std::runtime_error("State::getLonPosition longitudinal position not initialized");
    return lonPosition;
}

void State::setLonPosition(const double lonPos) {
    lonPosition = lonPos;
    validStates.lonPosition = true;
}

double State::getLatPosition() const {
    if (!validStates.latPosition)
        throw std::runtime_error("State::getLatPosition lateral position not initialized");
    return latPosition;
}

void State::setLatPosition(const double latPos) {
    latPosition = latPos;
    validStates.latPosition = true;
}

double State::getGlobalOrientation() const { return globalOrientation; }

void State::setGlobalOrientation(const double orientation) {
    globalOrientation = orientation;
    validStates.globalOrientation = true;
}

double State::getCurvilinearOrientation() const {
    if (!validStates.curvilinearOrientation)
        throw std::runtime_error("State::getCurvilinearOrientation curvilinear orientation not initialized");
    return curvilinearOrientation;
}

void State::setCurvilinearOrientation(const double orientation) {
    curvilinearOrientation = orientation;
    validStates.curvilinearOrientation = true;
}

size_t State::getTimeStep() const { return timeStep; }

void State::setTimeStep(const size_t time) { timeStep = time; }

const ValidStates &State::getValidStates() const { return validStates; }

vertex State::get2DVertex() { return {xPosition, yPosition}; }

std::string State::to_string() const {
    std::ostringstream oss;
    oss << "State["
        << "timeStep=" << timeStep << ", xPosition=" << xPosition << ", yPosition=" << yPosition
        << ", velocity=" << velocity << ", acceleration=" << acceleration << ", lonPosition=" << lonPosition
        << ", latPosition=" << latPosition << ", globalOrientation=" << globalOrientation
        << ", curvilinearOrientation=" << curvilinearOrientation << "]";
    return oss.str();
}
