//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "command_line_input.h"
#include "commonroad_cpp/world.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/road_network.h>

int main(int argc, char **argv) {

    // Read command line parameters; if none are provided, use default values (specified in read_command_line_values)
    int num_threads;
    std::string xmlFilePath;
    int error_code = CommandLine::readCommandLineValues(argc, argv, num_threads, xmlFilePath);
    if (error_code != 0)
        return error_code;

    // Read and parse CommonRoad scenario file
    const auto &[obstacles, roadNetwork] = CommandLine::getDataFromCommonRoad(xmlFilePath);

    auto world{World(0, roadNetwork, obstacles, {}, 0.1)};

    return 0;
}
