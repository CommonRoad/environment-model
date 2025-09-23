
#include "commonroad_cpp/obstacle/time_parameters.h"
#include <cassert>
#include <cstddef>
#include <string>

TimeParameters::TimeParameters(const size_t relevantHistorySize, const double reactionTime, const double timeStepSize)
    : relevantHistorySize{relevantHistorySize}, reactionTime{reactionTime}, timeStepSize(timeStepSize) {
    assert(reactionTime >= 0.0);
    assert(timeStepSize > 0.0);
}

void TimeParameters::setTimeStepSize(const double stepSize) { timeStepSize = stepSize; }

size_t TimeParameters::getRelevantHistorySize() const noexcept { return relevantHistorySize; }

double TimeParameters::getReactionTime() const noexcept { return reactionTime; }

double TimeParameters::getTimeStepSize() const noexcept { return timeStepSize; }

TimeParameters TimeParameters::dynamicDefaults() { return TimeParameters{50, 0.3, 0.1}; }

TimeParameters TimeParameters::staticDefaults() { return TimeParameters{0, 0.0, 0.1}; }

std::string TimeParameters::to_string() const {
    return "relevantHistorySize: " + std::to_string(relevantHistorySize) +
           ", reactionTime: " + std::to_string(reactionTime) + ", timeStepSize: " + std::to_string(timeStepSize);
}
