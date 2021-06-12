//
// Created by Sebastian Maierhofer on 12.06.21.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "preserves_traffic_flow_predicate.h"

bool PreservesTrafficFlow::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                             const std::shared_ptr<Obstacle> &obstacleK,
                                             const std::shared_ptr<Obstacle> &obstacleP) {
    //  v_max_lane = self._speed_limit_suggested(time_step, vehicle)
    //  v_type = self._get_type_speed_limit(vehicle.obstacle_type)
    //  v_max = min(vehicle.vehicle_param.get("road_condition_speed_limit"),
    //              vehicle.vehicle_param.get("fov_speed_limit"),
    //              vehicle.vehicle_param.get("braking_speed_limit"), v_max_lane, v_type)
    //  if operating_mode is OperatingMode.MONITOR:
    //  if v_max - vehicle.states_lon[time_step].v < self._traffic_rules_param.get("min_velocity_dif"):
    //  return True
    //  else:
    //  return False
    //  elif operating_mode is OperatingMode.CONSTRAINT:
    //  return Constraint([ConstraintType.VELOCITY], ConstraintRepresentation.LOWER,
    //      v_max - self._traffic_rules_param.get("min_velocity_dif"))
    //  elif operating_mode is OperatingMode.ROBUSTNESS:
    //  return vehicle.states_lon[time_step].v - v_max + self._traffic_rules_param.get("min_velocity_dif") - 1e-17
}

double PreservesTrafficFlow::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                              const std::shared_ptr<Obstacle> &obstacleK,
                                              const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("PreservesTrafficFlow does not support robust evaluation!");
}

Constraint PreservesTrafficFlow::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                      const std::shared_ptr<Obstacle> &obstacleK,
                                                      const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("PreservesTrafficFlow does not support constraint evaluation!");
}
