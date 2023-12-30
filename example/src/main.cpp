//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "command_line_input.h"
#include "predicate_manager.h"
#include <boost/stacktrace.hpp>
#include <iostream>

int main(int argc, char **argv) {

    // Read command line parameters; if none are provided, use default values (specified in read_command_line_values)
    int num_threads;
    std::string filePath;
    int error_code = InputUtils::readCommandLineValues(argc, argv, num_threads, filePath);
    if (error_code != 0)
        return error_code;
    try {
        PredicateManager eval{num_threads, filePath};
        eval.extractPredicateSatisfaction();
    } catch (...) {
        std::cout << boost::stacktrace::stacktrace();
    }
    return 0;
}
