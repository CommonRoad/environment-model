#pragma once

#include "obstacle_cache.h"
#include "signal_state.h"
#include "state.h"
#include <tsl/robin_map.h>

template <typename Value> using time_step_map_t = tsl::robin_map<time_step_t, Value>;
//** type of history/trajectory prediction maps for physical states */
using state_map_t = time_step_map_t<std::shared_ptr<State>>;
//** type of history/trajectory prediction maps for signal states*/
using signal_state_map_t = time_step_map_t<std::shared_ptr<SignalState>>;

/**
 * Struct representing trajectory prediction.
 */
struct TrajectoryPrediction {
    signal_state_map_t signalSeries{};  //**< signal series of the obstacle */
    state_map_t trajectoryPrediction{}; //**< trajectory prediction of the obstacle */
    ObstacleCache obstacleCache{};      //**< cache for trajectory prediction */

    /**
     * Resets helper mappings for obstacle time steps.
     *
     * @param timeStep Time step to remove from mapping variables.
     * @param clearReferenceLane Boolean indicating whether reference lane should be cleared.
     */
    void removeTimeStepFromMappingVariables(const size_t timeStep, const bool clearReferenceLane) {
        obstacleCache.removeTimeStepFromMappingVariables(timeStep, clearReferenceLane);
    }

    /**
     * Clears the cache for trajectory prediction.
     */
    void clearCache() { obstacleCache.clear(); }

    /**
     * Creates a string representation of a trajectory prediction.
     *
     * @return String representation of trajectory prediction.
     */
    std::string to_string() const {
        std::ostringstream oss;
        oss << "TrajectoryPrediction: ";
        oss << "  SignalSeries size: " << signalSeries.size()
            << ", TrajectoryPrediction size: " << trajectoryPrediction.size() << "\n";
        oss << "    SignalSeries: [";
        for (const auto &[_, snd] : signalSeries)
            oss << snd->to_string();
        oss << "]\n";
        oss << "    TrajectoryPrediction: [";
        for (const auto &[_, snd] : trajectoryPrediction)
            oss << snd->to_string();
        oss << "]\n";
        oss << obstacleCache.to_string();
        return oss.str();
    }
};
