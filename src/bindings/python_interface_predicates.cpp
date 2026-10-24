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

    nb::class_<Timer>(m, "Timer").def(nb::init<>());

    nb::class_<PredicateStatistics>(m, "PredicateStatistics")
        .def(nb::init<>())
        .def_ro("computation_time", &PredicateStatistics::computationTime)
        .def_ro("total_computation_time", &PredicateStatistics::totalComputationTime)
        .def_ro("num_executions", &PredicateStatistics::numExecutions)
        .def_ro("num_satisfaction", &PredicateStatistics::numSatisfaction)
        .def("reset", &PredicateStatistics::reset);

    nb::enum_<SetBasedMode>(m, "SetBasedMode")
        .value("none", SetBasedMode::none)
        .value("must", SetBasedMode::must)
        .value("may", SetBasedMode::may);

    using ObstaclePtr = const std::shared_ptr<Obstacle> &;
    using WorldPtr = const std::shared_ptr<World> &;
    using Parameters = const std::vector<std::string> &;
    nb::class_<CommonRoadPredicate>(m, "CommonRoadPredicate")
        .def("boolean_evaluation", &CommonRoadPredicate::simpleBooleanEvaluation, nb::arg("timeStep"), nb::arg("world"),
             nb::arg("obstacleK"), nb::arg("obstacleP"),
             nb::arg("additionalFunctionParameters") = std::vector<std::string>{"0.0"}, nb::arg("setBased") = false)
        .def("boolean_evaluation",
             nb::overload_cast<size_t, WorldPtr, ObstaclePtr, ObstaclePtr, Parameters, SetBasedMode>(
                 &CommonRoadPredicate::booleanEvaluation),
             nb::arg("timeStep"), nb::arg("world"), nb::arg("obstacleK"), nb::arg("obstacleP"),
             nb::arg("additionalFunctionParameters"), nb::arg("mode"))
        .def("statistic_boolean_evaluation",
             nb::overload_cast<size_t, WorldPtr, ObstaclePtr, const std::shared_ptr<Timer> &,
                               const std::shared_ptr<PredicateStatistics> &, ObstaclePtr, Parameters, bool>(
                 &CommonRoadPredicate::statisticBooleanEvaluation),
             nb::arg("timeStep"), nb::arg("world"), nb::arg("obstacleK"), nb::arg("evaluationTimer"),
             nb::arg("statistics"), nb::arg("obstacleP"),
             nb::arg("additionalFunctionParameters") = std::vector<std::string>{"0.0"}, nb::arg("setBased") = false)
        .def("statistic_boolean_evaluation",
             nb::overload_cast<size_t, WorldPtr, ObstaclePtr, const std::shared_ptr<Timer> &,
                               const std::shared_ptr<PredicateStatistics> &, ObstaclePtr, Parameters, SetBasedMode>(
                 &CommonRoadPredicate::statisticBooleanEvaluation),
             nb::arg("timeStep"), nb::arg("world"), nb::arg("obstacleK"), nb::arg("evaluationTimer"),
             nb::arg("statistics"), nb::arg("obstacleP"), nb::arg("additionalFunctionParameters"), nb::arg("mode"))
        .def("robust_evaluation",
             nb::overload_cast<size_t, WorldPtr, ObstaclePtr, ObstaclePtr, Parameters, bool>(
                 &CommonRoadPredicate::robustEvaluation),
             nb::arg("timeStep"), nb::arg("world"), nb::arg("obstacleK"), nb::arg("obstacleP"),
             nb::arg("additionalFunctionParameters") = std::vector<std::string>{"0.0"}, nb::arg("setBased") = false)
        .def("robust_evaluation",
             nb::overload_cast<size_t, WorldPtr, ObstaclePtr, ObstaclePtr, Parameters, SetBasedMode>(
                 &CommonRoadPredicate::robustEvaluation),
             nb::arg("timeStep"), nb::arg("world"), nb::arg("obstacleK"), nb::arg("obstacleP"),
             nb::arg("additionalFunctionParameters"), nb::arg("mode"))
        .def("constraint_evaluation",
             nb::overload_cast<size_t, WorldPtr, ObstaclePtr, ObstaclePtr, Parameters, bool>(
                 &CommonRoadPredicate::constraintEvaluation),
             nb::arg("timeStep"), nb::arg("world"), nb::arg("obstacleK"), nb::arg("obstacleP"),
             nb::arg("additionalFunctionParameters") = std::vector<std::string>{"0.0"}, nb::arg("setBased") = false)
        .def("constraint_evaluation",
             nb::overload_cast<size_t, WorldPtr, ObstaclePtr, ObstaclePtr, Parameters, SetBasedMode>(
                 &CommonRoadPredicate::constraintEvaluation),
             nb::arg("timeStep"), nb::arg("world"), nb::arg("obstacleK"), nb::arg("obstacleP"),
             nb::arg("additionalFunctionParameters"), nb::arg("mode"))
        .def_prop_ro("is_vehicle_dependent", &CommonRoadPredicate::isVehicleDependent);

    instantiate_predicates(m);

    m.def("lookup_predicate", lookup_predicate);
    m.def("predicates", get_predicates);
}
