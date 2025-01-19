#include <memory>
#include <string>

#include <nanobind/nanobind.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "commonroad_cpp/auxiliaryDefs/structs.h"
#include "python_interface_predicates.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/commonroad_predicate.h>
#include <commonroad_cpp/world.h>

namespace nb = nanobind;

void instantiate_predicates(nb::module_ &m);

static std::shared_ptr<CommonRoadPredicate> lookup_predicate(const std::string &name) { return predicates[name]; }

static decltype(predicates) get_predicates() { return predicates; }

void init_python_interface_predicates(nb::module_ &m) {
    nb::class_<PredicateParameters>(m, "PredicateParameters");

    nb::class_<Constraint>(m, "Constraint").def_rw("real_valued_constraint", &Constraint::realValuedConstraint);

    nb::class_<CommonRoadPredicate>(m, "CommonRoadPredicate")
        .def("boolean_evaluation", &CommonRoadPredicate::simpleBooleanEvaluation, nb::arg("timeStep"), nb::arg("world"),
             nb::arg("obstacleK"), nb::arg("obstacleP"),
             nb::arg("additionalFunctionParameters") = std::vector<std::string>{"0.0"}, nb::arg("setBased") = false)
        .def("robust_evaluation", &CommonRoadPredicate::robustEvaluation, nb::arg("timeStep"), nb::arg("world"),
             nb::arg("obstacleK"), nb::arg("obstacleP"),
             nb::arg("additionalFunctionParameters") = std::vector<std::string>{"0.0"}, nb::arg("setBased") = false)
        .def("constraint_evaluation", &CommonRoadPredicate::constraintEvaluation, nb::arg("timeStep"), nb::arg("world"),
             nb::arg("obstacleK"), nb::arg("obstacleP"),
             nb::arg("additionalFunctionParameters") = std::vector<std::string>{"0.0"}, nb::arg("setBased") = false);

    instantiate_predicates(m);

    m.def("lookup_predicate", lookup_predicate);
    m.def("predicates", get_predicates);
}
