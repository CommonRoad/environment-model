#include <memory>
#include <string>

#include "python_interface_predicates.h"
#include <commonroad_cpp/predicates/commonroad_predicate.h>

namespace nb = nanobind;

void instantiate_predicates(nb::module_ &m);

static std::shared_ptr<CommonRoadPredicate> lookup_predicate(const std::string &name) { return predicates[name]; }

static decltype(predicates) get_predicates() { return predicates; }

void init_python_interface_predicates(nb::module_ &m) {
    nb::class_<OptionalPredicateParameters>(m, "OptionalPredicateParameters");

    nb::class_<CommonRoadPredicate>(m, "CommonRoadPredicate")
        .def("boolean_evaluation", &CommonRoadPredicate::simpleBooleanEvaluation)
        .def("robust_evaluation", &CommonRoadPredicate::robustEvaluation)
        .def("constraint_evaluation", &CommonRoadPredicate::constraintEvaluation);

    instantiate_predicates(m);

    m.def("lookup_predicate", lookup_predicate);
    m.def("predicates", get_predicates);
}