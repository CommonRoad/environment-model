#pragma once

#include "obstacle_cache.h"
#include "signal_state.h"
#include "state.h"
#include <tsl/robin_map.h>

template <typename Value> using time_step_map_t = tsl::robin_map<time_step_t, Value>;
//** type of history/trajectory prediction maps for physical states */
using state_map_t = time_step_map_t<std::shared_ptr<State>>;
//** type of prediction maps for occupancies */
using occupancy_map_t = time_step_map_t<std::shared_ptr<Occupancy>>;
//** type of history/trajectory prediction maps for signal states*/
using signal_state_map_t = time_step_map_t<std::shared_ptr<SignalState>>;

/**
 * Struct representing recorded states.
 */
struct RecordedStates {
    std::shared_ptr<State> currentState;             //**< pointer to current state of obstacle */
    std::shared_ptr<SignalState> currentSignalState; //**< pointer to current signal state of obstacle */
    state_map_t trajectoryHistory{};                 //**< previous states of the obstacle */
    signal_state_map_t signalSeriesHistory{};        //**< previous signal states of the obstacle */
    ObstacleCache occupancyRecorded;                 //**< cache for recorded occupancy (history + current time step) */

    /**
     * Resets helper mappings for obstacle time steps.
     *
     * @param timeStep Time step to remove from mapping variables.
     * @param clearReferenceLane Boolean indicating whether reference lane should be cleared.
     */
    void removeTimeStepFromMappingVariables(const size_t timeStep, const bool clearReferenceLane) {
        occupancyRecorded.removeTimeStepFromMappingVariables(timeStep, clearReferenceLane);
    }

    /**
     * Clears the cache for trajectory prediction.
     */
    void clearCache() { occupancyRecorded.clear(); }

    /**
     * Creates a string representation of a recorded states.
     *
     * @return String representation of recorded states.
     */
    std::string to_string() const {
        std::ostringstream oss;
        oss << "RecordedStates: ";
        oss << "CurrentState: " << (currentState ? currentState->to_string() : "nullptr")
            << ", CurrentSignalState: " << (currentSignalState ? currentSignalState->to_string() : "nullptr") << "\n";
        oss << "TrajectoryHistory size: " << trajectoryHistory.size()
            << ", SignalSeriesHistory size: " << signalSeriesHistory.size() << "\n";
        oss << "TrajectoryHistory: [";
        for (size_t i = 0; i < trajectoryHistory.size(); ++i) {
            if (trajectoryHistory.at(i)) {
                oss << trajectoryHistory.at(i)->to_string();
            } else {
                oss << "nullptr";
            }
            if (i < trajectoryHistory.size() - 1)
                oss << ", ";
        }
        oss << "]\n";
        oss << "SignalSeriesHistory: [";
        for (size_t i = 0; i < signalSeriesHistory.size(); ++i) {
            if (signalSeriesHistory.at(i)) {
                oss << signalSeriesHistory.at(i)->to_string();
            } else {
                oss << "nullptr";
            }
            if (i < signalSeriesHistory.size() - 1)
                oss << ", ";
        }
        oss << "]\n";
        oss << "OccupancyRecorded: [" << occupancyRecorded.to_string() << "]\n";
        return oss.str();
    }
};
