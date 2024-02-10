#include <cassert>
#include <commonroad_cpp/predicates/predicate_parameter.h>

PredicateParam::PredicateParam(const std::string &name, const std::string &description, double max, double min,
                               const std::vector<std::string> &occurrences, const std::string &aProperty,
                               const std::string &usage, const std::string &type, const std::string &unit, double value)
    : name(name), description(description), max(max), min(min), occurrences(occurrences), property(aProperty),
      usage(usage), type(type), unit(unit), value(value) {}

void PredicateParam::updateValue(double val) { this->value = val; }

double PredicateParam::getValue() { return value; }
void PredicateParam::checkParameterValidity() {
    assert(value < max);
    assert(value > min);
}