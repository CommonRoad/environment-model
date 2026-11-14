#include <commonroad_cpp/obstacle/obstacle_cache.h>

void ObstacleCache::removeTimeStepFromMappingVariables(const size_t timeStep, const bool clearReferenceLane) {
    occupiedLanelets.erase(timeStep);
    occupiedLaneletsState.erase(timeStep);
    occupiedLaneletsFront.erase(timeStep);
    occupiedLaneletsBack.erase(timeStep);
    occupiedLanesDrivingDir.erase(timeStep);
    occupiedLaneletsDrivingDir.erase(timeStep);
    occupiedLaneletsNotDrivingDir.erase(timeStep);
    if (clearReferenceLane)
        referenceLane.erase(timeStep);
    occupiedLanes.erase(timeStep);
    frontXYPositions.erase(timeStep);
    backXYPositions.erase(timeStep);
    leftLatPosition.erase(timeStep);
    rightLatPosition.erase(timeStep);
    lateralDistanceToObjects.erase(timeStep);
    convertedPositions.erase(timeStep);
    shapeAtTimeStep.erase(timeStep);
}

void ObstacleCache::clear() {
    occupiedLanelets.clear();
    occupiedLaneletsState.clear();
    occupiedLaneletsFront.clear();
    occupiedLaneletsBack.clear();
    occupiedLanesDrivingDir.clear();
    occupiedLaneletsDrivingDir.clear();
    occupiedLaneletsNotDrivingDir.clear();
    referenceLane.clear();
    occupiedLanes.clear();
    frontXYPositions.clear();
    backXYPositions.clear();
    leftLatPosition.clear();
    rightLatPosition.clear();
    lateralDistanceToObjects.clear();
    convertedPositions.clear();
    shapeAtTimeStep.clear();
}

std::string ObstacleCache::to_string() const {
    std::ostringstream oss;
    oss << "    ObstacleCache: ";
    // occupiedLanelets
    oss << "\n      occupiedLanelets: " << occupiedLanelets.size() << " entries";
    for (const auto &[ts, lanelets] : occupiedLanelets) {
        oss << "\n        t=" << ts << ": [";
        for (size_t i = 0; i < lanelets.size(); ++i) {
            oss << lanelets[i]->getId();
            if (i < lanelets.size() - 1)
                oss << ", ";
        }
        oss << "]";
    }
    // occupiedLaneletsState
    oss << "\n      occupiedLaneletsState: " << occupiedLaneletsState.size() << " entries";
    for (const auto &[ts, lanelets] : occupiedLaneletsState) {
        oss << "\n        t=" << ts << ": [";
        for (size_t i = 0; i < lanelets.size(); ++i) {
            oss << lanelets[i]->getId();
            if (i < lanelets.size() - 1)
                oss << ", ";
        }
        oss << "]";
    }
    // occupiedLaneletsFront
    oss << "\n      occupiedLaneletsFront: " << occupiedLaneletsFront.size() << " entries";
    for (const auto &[ts, lanelets] : occupiedLaneletsFront) {
        oss << "\n        t=" << ts << ": [";
        for (size_t i = 0; i < lanelets.size(); ++i) {
            oss << lanelets[i]->getId();
            if (i < lanelets.size() - 1)
                oss << ", ";
        }
        oss << "]";
    }
    // occupiedLaneletsBack
    oss << "\n      occupiedLaneletsBack: " << occupiedLaneletsBack.size() << " entries";
    for (const auto &[ts, lanelets] : occupiedLaneletsBack) {
        oss << "\n        t=" << ts << ": [";
        for (size_t i = 0; i < lanelets.size(); ++i) {
            oss << lanelets[i]->getId();
            if (i < lanelets.size() - 1)
                oss << ", ";
        }
        oss << "]";
    }
    // occupiedLanesDrivingDir
    oss << "\n      occupiedLanesDrivingDir: " << occupiedLanesDrivingDir.size() << " entries";
    for (const auto &[ts, lanes] : occupiedLanesDrivingDir) {
        oss << "\n        t=" << ts << ": [";
        for (size_t i = 0; i < lanes.size(); ++i) {
            oss << lanes[i]->getId();
            if (i < lanes.size() - 1)
                oss << ", ";
        }
        oss << "]";
    }
    // occupiedLaneletsDrivingDir
    oss << "\n      occupiedLaneletsDrivingDir: " << occupiedLaneletsDrivingDir.size() << " entries";
    for (const auto &[ts, lanelets] : occupiedLaneletsDrivingDir) {
        oss << "\n        t=" << ts << ": [";
        for (size_t i = 0; i < lanelets.size(); ++i) {
            oss << lanelets[i]->getId();
            if (i < lanelets.size() - 1)
                oss << ", ";
        }
        oss << "]";
    }
    // occupiedLaneletsNotDrivingDir
    oss << "\n      occupiedLaneletsNotDrivingDir: " << occupiedLaneletsNotDrivingDir.size() << " entries";
    for (const auto &[ts, lanelets] : occupiedLaneletsNotDrivingDir) {
        oss << "\n        t=" << ts << ": [";
        for (size_t i = 0; i < lanelets.size(); ++i) {
            oss << lanelets[i]->getId();
            if (i < lanelets.size() - 1)
                oss << ", ";
        }
        oss << "]";
    }
    // referenceLane
    oss << "\n      referenceLane: " << referenceLane.size() << " entries";
    for (const auto &[ts, lane] : referenceLane) {
        oss << "\n        t=" << ts << ": " << lane->getId() << "";
    }
    // occupiedLanes
    oss << "\n      occupiedLanes: " << occupiedLanes.size() << " entries";
    for (const auto &[ts, lanes] : occupiedLanes) {
        oss << "\n        t=" << ts << ": [";
        for (size_t i = 0; i < lanes.size(); ++i) {
            oss << lanes[i]->getId();
            if (i < lanes.size() - 1)
                oss << ", ";
        }
        oss << "]";
    }
    // frontXYPositions
    oss << "\n      frontXYPositions: " << frontXYPositions.size() << " entries";
    for (const auto &[ts, positions] : frontXYPositions) {
        oss << "\n        t=" << ts << ": [";
        for (size_t i = 0; i < positions.size(); ++i) {
            oss << positions[i];
            if (i < positions.size() - 1)
                oss << ", ";
        }
        oss << "]";
    }
    // backXYPositions
    oss << "\n      backXYPositions: " << backXYPositions.size() << " entries";
    for (const auto &[ts, positions] : backXYPositions) {
        oss << "\n        t=" << ts << ": [";
        for (size_t i = 0; i < positions.size(); ++i) {
            oss << positions[i];
            if (i < positions.size() - 1)
                oss << ", ";
        }
        oss << "]";
    }
    // leftLatPosition
    oss << "\n      leftLatPosition: " << leftLatPosition.size() << " entries";
    for (const auto &[ts, val] : leftLatPosition) {
        oss << "\n        t=" << ts << ": " << val;
    }
    // rightLatPosition
    oss << "\n      rightLatPosition: " << rightLatPosition.size() << " entries";
    for (const auto &[ts, val] : rightLatPosition) {
        oss << "\n    t=" << ts << ": " << val;
    }
    // lateralDistanceToObjects
    oss << "\n      lateralDistanceToObjects: " << lateralDistanceToObjects.size() << " entries";
    for (const auto &[ts, distMap] : lateralDistanceToObjects) {
        oss << "\n        t=" << ts << ": {";
        size_t count = 0;
        for (const auto &[oid, dist] : distMap) {
            oss << oid << ":" << dist;
            if (++count < distMap.size())
                oss << ", ";
        }
        oss << "}";
    }
    // convertedPositions
    oss << "\n      convertedPositions: " << convertedPositions.size() << " entries";
    for (const auto &[ts, ccsMap] : convertedPositions) {
        oss << "\n        t=" << ts << ": {";
        size_t count = 0;
        for (const auto &[ccs, pos] : ccsMap) {
            oss << "CCS@" << ccs.get() << ": [" << pos[0] << ", " << pos[1] << ", " << pos[2] << "]";
            if (++count < ccsMap.size())
                oss << ", ";
        }
        oss << "}";
    }
    // shapeAtTimeStep
    oss << "\n      shapeAtTimeStep: " << shapeAtTimeStep.size() << " entries";
    for (const auto &[ts, mp] : shapeAtTimeStep) {
        oss << "\n        t=" << ts << ": polygons=" << mp.size() << "";
    }
    return oss.str();
}
