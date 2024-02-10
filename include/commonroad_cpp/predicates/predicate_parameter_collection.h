#pragma once

// Note: This include needs to come first in order to have M_PI defined
// See https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <math.h>

#include <array>
#include <commonroad_cpp/auxiliaryDefs/structs.h>
#include <commonroad_cpp/predicates/predicate_parameter.h>
#include <cstdint>
#include <limits>
#include <map>
#include <string>

extern std::map<std::string, PredicateParam> paramMap;

struct PredicateParameters {
    /**
     * Constructor of predicate parameters colllection.
     */
    PredicateParameters() { checkParameterValidity(); }

    /**
     * Checks validity of all parameters.
     */
    void checkParameterValidity();

    /**
     * Updates single predicate parameter
     *
     * @param name (string) name of parameter
     * @param value (double) value to be set
     */
    void updateParam(const std::string &name, double value);

    /**
     * Getter for predicate parameter
     *
     * @param name (string) name of parameter
     */
    double getParam(const std::string &name);

  private:
    std::map<std::string, double> constantMap{
        {"epsilon", 1e-6},         // small value close to zero for different purposes
        {"fovSpeedLimit", 50},     // field of view speed limit; will be replaced by compute with calc_v_max_fov() [m/s]
        {"brakingSpeedLimit", 43}, // braking speed limit; will be replaced by compute with calc_v_max_braking() [m/s]
        {"roadConditionSpeedLimit",
         50}, // road condition speed limit; will be replaced by compute with calc_v_max_road_condition() [m/s]
    };
    std::map<std::string, PredicateParam> parameterCollection = paramMap;
};