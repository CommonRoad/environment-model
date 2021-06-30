//
// Created by Sebastian Maierhofer on 28.10.20.
//

#include "command_line_input.h"
//#include "../../predicates/predicates.h"

#include <chrono>

int main(int argc, char **argv) {

    // Read command line parameters; if none are provided, use default values (specified in read_command_line_values)
    int num_threads;
    std::string xmlFilePath;
    int error_code = CommandLine::readCommandLineValues(argc, argv, num_threads, xmlFilePath);
    if (error_code != 0)
        return error_code;

    // Read and parse CommonRoad scenario file
    const auto &[obstacles, roadNetwork] = CommandLine::getDataFromCommonRoad(xmlFilePath);

    auto world{World(0, roadNetwork, obstacles, {})};

    return 0;
}
