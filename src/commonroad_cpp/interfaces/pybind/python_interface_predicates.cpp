//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <memory>
#include <string>

#include <commonroad_cpp/interfaces/commonroad/xml_reader.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/commonroad_predicate.h>

#include <commonroad_cpp/obstacle/obstacle.h>

#include "python_interface_predicates.h"

#include <pybind11/stl.h> // for vector

namespace py = pybind11;

void instantiate_predicates(py::module_ &m);

static std::shared_ptr<CommonRoadPredicate> lookup_predicate(const std::string &name) { return predicates[name]; }

static decltype(predicates) get_predicates() { return predicates; }

void init_python_interface_predicates(py::module_ &m) {
    py::class_<OptionalPredicateParameters, std::shared_ptr<OptionalPredicateParameters>>(
        m, "OptionalPredicateParameters");

    py::class_<CommonRoadPredicate, std::shared_ptr<CommonRoadPredicate>>(m, "CommonRoadPredicate")
        .def("boolean_evaluation", &CommonRoadPredicate::simpleBooleanEvaluation)
        .def("robust_evaluation", &CommonRoadPredicate::robustEvaluation)
        .def("constraint_evaluation", &CommonRoadPredicate::constraintEvaluation);

    instantiate_predicates(m);

    m.def("lookup_predicate", lookup_predicate);
    m.def("predicates", get_predicates);
}