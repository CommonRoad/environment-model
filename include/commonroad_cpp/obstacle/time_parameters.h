#pragma once

#include <cassert>
#include <cstddef>

/**
 * TimeParameters includes information regarding obstacle time parameters
 */
class TimeParameters {
    size_t relevantHistorySize{50}; //**< number of history time steps to consider  */
    double reactionTime{0.3};       /** reaction time of obstacle in [s] */

  public:
    TimeParameters() = default;

    /**
     * Complete constructor for SensorParameters.
     *
     * @param relevantHistorySize length of history to consider
     * @param reactionTime reaction time of obstacle in [s]
     */
    TimeParameters(size_t relevantHistorySize, double reactionTime)
        : relevantHistorySize{relevantHistorySize}, reactionTime{reactionTime} {
        assert(reactionTime >= 0.0);
    }

    /**
     * Getter for relevant history.
     *
     * @return Size of relevant history.
     */
    [[nodiscard]] size_t getRelevantHistorySize() const noexcept { return relevantHistorySize; }

    /**
     * Getter for reaction time.
     *
     * @return Reaction time [s].
     */
    [[nodiscard]] double getReactionTime() const noexcept { return reactionTime; }

    /**
     * Default time parameters for dynamic obstacles: relevantHistorySize = 50, reactionTime = 0.3s.
     *
     * @return Default dynamic obstacle sensor parameters.
     */
    static TimeParameters dynamicDefaults() { return TimeParameters{50, 0.3}; }

    /**
     * Default time parameters for static obstacles: relevantHistorySize = 0, reactionTime = 0.0s.
     *
     * @return Default static obstacle sensor parameters.
     */
    static TimeParameters staticDefaults() { return TimeParameters{0, 0.0}; }
};
