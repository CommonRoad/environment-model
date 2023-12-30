#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "regulatory_elements.h"
#include "types_and_definitions.h"

using TrafficSignTable = const std::unordered_map<TrafficSignTypes, std::string>;

extern TrafficSignTable TrafficSignIDGermany;
extern TrafficSignTable TrafficSignIDUSA;
extern TrafficSignTable TrafficSignIDSpain;
extern TrafficSignTable TrafficSignIDArgentina;
extern TrafficSignTable TrafficSignIDBelgium;

extern const std::unordered_map<std::string, TrafficSignTypes> TrafficSignNames;

extern const std::unordered_map<SupportedTrafficSignCountry, TrafficSignTable *> TrafficSignLookupTableByCountry;

extern const std::unordered_map<std::string, std::vector<int>> priorityTable;

extern const std::unordered_map<std::string, TurningDirection> TurningDirectionNames;

extern const std::unordered_map<std::string, TrafficLightState> TrafficLightStateNames;
