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

// #include "python_interface.h"

#include "python_interface_core.h"
#include "python_interface_legacy.h"
#include "python_interface_predicates.h"

namespace py = pybind11;

PYBIND11_MODULE(crcpp, m) {
    m.doc() = "CommonRoad Python/C++ Interface";

    init_python_interface_legacy(m);

    py::module_ m_predicates = m.def_submodule("predicates");
    init_python_interface_predicates(m_predicates);

    init_python_interface_core(m);
}