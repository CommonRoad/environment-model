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
    oss << "ObstacleCache: \n";
    // occupiedLanelets
    oss << "  occupiedLanelets: " << occupiedLanelets.size() << " entries\n";
    for (const auto &[ts, lanelets] : occupiedLanelets) {
        oss << "    t=" << ts << ": [";
        for (size_t i = 0; i < lanelets.size(); ++i) {
            oss << lanelets[i]->getId();
            if (i < lanelets.size() - 1)
                oss << ", ";
        }
        oss << "]\n";
    }
    // occupiedLaneletsState
    oss << "  occupiedLaneletsState: " << occupiedLaneletsState.size() << " entries\n";
    for (const auto &[ts, lanelets] : occupiedLaneletsState) {
        oss << "    t=" << ts << ": [";
        for (size_t i = 0; i < lanelets.size(); ++i) {
            oss << lanelets[i]->getId();
            if (i < lanelets.size() - 1)
                oss << ", ";
        }
        oss << "]\n";
    }
    // occupiedLaneletsFront
    oss << "  occupiedLaneletsFront: " << occupiedLaneletsFront.size() << " entries\n";
    for (const auto &[ts, lanelets] : occupiedLaneletsFront) {
        oss << "    t=" << ts << ": [";
        for (size_t i = 0; i < lanelets.size(); ++i) {
            oss << lanelets[i]->getId();
            if (i < lanelets.size() - 1)
                oss << ", ";
        }
        oss << "]\n";
    }
    // occupiedLaneletsBack
    oss << "  occupiedLaneletsBack: " << occupiedLaneletsBack.size() << " entries\n";
    for (const auto &[ts, lanelets] : occupiedLaneletsBack) {
        oss << "    t=" << ts << ": [";
        for (size_t i = 0; i < lanelets.size(); ++i) {
            oss << lanelets[i]->getId();
            if (i < lanelets.size() - 1)
                oss << ", ";
        }
        oss << "]\n";
    }
    // occupiedLanesDrivingDir
    oss << "  occupiedLanesDrivingDir: " << occupiedLanesDrivingDir.size() << " entries\n";
    for (const auto &[ts, lanes] : occupiedLanesDrivingDir) {
        oss << "    t=" << ts << ": [";
        for (size_t i = 0; i < lanes.size(); ++i) {
            oss << lanes[i]->getId();
            if (i < lanes.size() - 1)
                oss << ", ";
        }
        oss << "]\n";
    }
    // occupiedLaneletsDrivingDir
    oss << "  occupiedLaneletsDrivingDir: " << occupiedLaneletsDrivingDir.size() << " entries\n";
    for (const auto &[ts, lanelets] : occupiedLaneletsDrivingDir) {
        oss << "    t=" << ts << ": [";
        for (size_t i = 0; i < lanelets.size(); ++i) {
            oss << lanelets[i]->getId();
            if (i < lanelets.size() - 1)
                oss << ", ";
        }
        oss << "]\n";
    }
    // occupiedLaneletsNotDrivingDir
    oss << "  occupiedLaneletsNotDrivingDir: " << occupiedLaneletsNotDrivingDir.size() << " entries\n";
    for (const auto &[ts, lanelets] : occupiedLaneletsNotDrivingDir) {
        oss << "    t=" << ts << ": [";
        for (size_t i = 0; i < lanelets.size(); ++i) {
            oss << lanelets[i]->getId();
            if (i < lanelets.size() - 1)
                oss << ", ";
        }
        oss << "]\n";
    }
    // referenceLane
    oss << "  referenceLane: " << referenceLane.size() << " entries\n";
    for (const auto &[ts, lane] : referenceLane) {
        oss << "    t=" << ts << ": " << lane->getId() << "\n";
    }
    // occupiedLanes
    oss << "  occupiedLanes: " << occupiedLanes.size() << " entries\n";
    for (const auto &[ts, lanes] : occupiedLanes) {
        oss << "    t=" << ts << ": [";
        for (size_t i = 0; i < lanes.size(); ++i) {
            oss << lanes[i]->getId();
            if (i < lanes.size() - 1)
                oss << ", ";
        }
        oss << "]\n";
    }
    // frontXYPositions
    oss << "  frontXYPositions: " << frontXYPositions.size() << " entries\n";
    for (const auto &[ts, positions] : frontXYPositions) {
        oss << "    t=" << ts << ": [";
        for (size_t i = 0; i < positions.size(); ++i) {
            oss << positions[i];
            if (i < positions.size() - 1)
                oss << ", ";
        }
        oss << "]\n";
    }
    // backXYPositions
    oss << "  backXYPositions: " << backXYPositions.size() << " entries\n";
    for (const auto &[ts, positions] : backXYPositions) {
        oss << "    t=" << ts << ": [";
        for (size_t i = 0; i < positions.size(); ++i) {
            oss << positions[i];
            if (i < positions.size() - 1)
                oss << ", ";
        }
        oss << "]\n";
    }
    // leftLatPosition
    oss << "  leftLatPosition: " << leftLatPosition.size() << " entries\n";
    for (const auto &[ts, val] : leftLatPosition) {
        oss << "    t=" << ts << ": " << val << "\n";
    }
    // rightLatPosition
    oss << "  rightLatPosition: " << rightLatPosition.size() << " entries\n";
    for (const auto &[ts, val] : rightLatPosition) {
        oss << "    t=" << ts << ": " << val << "\n";
    }
    // lateralDistanceToObjects
    oss << "  lateralDistanceToObjects: " << lateralDistanceToObjects.size() << " entries\n";
    for (const auto &[ts, distMap] : lateralDistanceToObjects) {
        oss << "    t=" << ts << ": {";
        size_t count = 0;
        for (const auto &[oid, dist] : distMap) {
            oss << oid << ":" << dist;
            if (++count < distMap.size())
                oss << ", ";
        }
        oss << "}\n";
    }
    // convertedPositions
    oss << "  convertedPositions: " << convertedPositions.size() << " entries\n";
    for (const auto &[ts, ccsMap] : convertedPositions) {
        oss << "    t=" << ts << ": {";
        size_t count = 0;
        for (const auto &[ccs, pos] : ccsMap) {
            oss << "CCS@" << ccs.get() << ": [" << pos[0] << ", " << pos[1] << ", " << pos[2] << "]";
            if (++count < ccsMap.size())
                oss << ", ";
        }
        oss << "}\n";
    }
    // shapeAtTimeStep
    oss << "  shapeAtTimeStep: " << shapeAtTimeStep.size() << " entries\n";
    for (const auto &[ts, mp] : shapeAtTimeStep) {
        oss << "    t=" << ts << ": polygons=" << mp.size() << "\n";
    }
    return oss.str();
}
