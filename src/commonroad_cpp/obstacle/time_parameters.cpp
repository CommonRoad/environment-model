
#include "commonroad_cpp/obstacle/time_parameters.h"
#include <cassert>
#include <cstddef>

TimeParameters::TimeParameters(size_t relevantHistorySize, double reactionTime)
    : relevantHistorySize{relevantHistorySize}, reactionTime{reactionTime} {
    assert(reactionTime >= 0.0);
}

size_t TimeParameters::getRelevantHistorySize() const noexcept { return relevantHistorySize; }

double TimeParameters::getReactionTime() const noexcept { return reactionTime; }

TimeParameters TimeParameters::dynamicDefaults() { return TimeParameters{50, 0.3}; }

TimeParameters TimeParameters::staticDefaults() { return TimeParameters{0, 0.0}; }
