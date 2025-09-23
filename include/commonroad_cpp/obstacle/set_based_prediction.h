#pragma once

#include "obstacle_cache.h"
#include "state.h"
#include <tsl/robin_map.h>

template <typename Value> using time_step_map_t = tsl::robin_map<time_step_t, Value>;
//** type of history/trajectory prediction maps for physical states */
using state_map_t = time_step_map_t<std::shared_ptr<State>>;
//** type of prediction maps for occupancies */
using occupancy_map_t = time_step_map_t<std::shared_ptr<Occupancy>>;

/**
 * Struct representing set-based prediction.
 */
struct SetBasedPrediction {
    occupancy_map_t setBasedPrediction{}; //**< set-based prediction of the obstacle */
    ObstacleCache obstacleCache{};        //**< cache for set-based prediction */

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
     * Clears the cache for set-based prediction.
     */
    void clearCache() { obstacleCache.clear(); }
};
