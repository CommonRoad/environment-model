//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "string"
#include "types_and_definitions.h"
#include "vector"
#include <climits>
#include <cstddef>
#include <utility>

struct TrafficLightCycleElement {
    TrafficLightState color;
    size_t duration;
};

struct vertex {
    double x{0};
    double y{0};
    vertex() = default;
    vertex operator+(const vertex &vert) const { return {this->x + vert.x, this->y + vert.y}; }
    vertex operator-(const vertex &vert) const { return {this->x - vert.x, this->y - vert.y}; }
    vertex operator*(const vertex &vert) const { return {this->x * vert.x, this->y * vert.y}; }
    vertex operator*(const double &scalar) const { return {this->x * scalar, this->y * scalar}; }
    void operator+=(const vertex &vert) {
        this->x += vert.x;
        this->y += vert.y;
    }
    void operator-=(const vertex &vert) {
        this->x -= vert.x;
        this->y -= vert.y;
    }
};

struct ValidStates {
    bool xPosition;
    bool yPosition;
    bool velocity;
    bool acceleration;
    bool lonPosition;
    bool latPosition;
    bool globalOrientation;
    bool curvilinearOrientation;
};

struct PredicateStatistics {
    long maxComputationTime{LONG_MIN};
    size_t minComputationTime{LONG_MAX};
    size_t totalComputationTime{0};
    size_t numExecutions{0};
    size_t numSatisfaction{0};
};

struct Constraint {
    double realValuedConstraint;
};

struct PredicateSatisfaction {
    PredicateSatisfaction(double costs, double satisfaction_probability)
        : costs(costs), satisfactionProbability(satisfaction_probability) {}
    double costs{100.0};
    double satisfactionProbability{0.0};
};

struct SimulationParameters {
    SimulationParameters(std::vector<std::string> directoryPaths, size_t egoVehicleId, std::string benchmarkId,
                         EvaluationMode evaluationMode, bool performanceMeasurement, std::string outputDirectory,
                         std::string outputFileName)
        : directoryPaths(std::move(directoryPaths)), egoVehicleId(egoVehicleId), benchmarkId(std::move(benchmarkId)),
          evaluationMode(evaluationMode), performanceMeasurement(performanceMeasurement),
          outputDirectory(std::move(outputDirectory)), outputFileName(std::move(outputFileName)){};
    SimulationParameters() = default;
    std::vector<std::string> directoryPaths; //**< List of directories in which all scenarios should be evaluated */
    size_t egoVehicleId{0};                  //**< ID of ego vehicle */
    std::string benchmarkId;                 //**< CommonRoad benchmark ID */
    EvaluationMode evaluationMode; //**< Evaluation mode which should be used, e.g., directory, single vehicle, ... */
    bool performanceMeasurement;   //**< Flag indicating whether performance should me measured. */
    std::string outputDirectory;   //**< Path to output directory of file to generate. */
    std::string outputFileName;    //**< name and file type for to generate. */
};

struct EgoVehicleParameters {
    EgoVehicleParameters(double fovSpeedLimit, double brakingSpeedLimit)
        : fovSpeedLimit(fovSpeedLimit), brakingSpeedLimit(brakingSpeedLimit){};
    EgoVehicleParameters() = default;
    double fovSpeedLimit{50}; //**< compute with calc_v_max_fov(ego_vehicle_param, simulation_param) */
    double brakingSpeedLimit{
        43}; //**< compute with calc_v_max_braking(ego_vehicle_param, simulation_param, traffic_rule_param) */
    double roadConditionSpeedLimit{50};
};
