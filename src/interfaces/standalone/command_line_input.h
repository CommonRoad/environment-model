//
// Created by Sebastian Maierhofer on 02.11.20.
//

#ifndef ENV_MODEL_COMMAND_LINE_INPUT_H
#define ENV_MODEL_COMMAND_LINE_INPUT_H

#include <iostream>

namespace CommandLine{
    /**
     * Reads the arguments provided via command line.
     *
     * @param argc Argument count: Number of arguments.
     * @param argv Argument vector: List of arguments. (Argument at position 0 is the executable)
     * @param num_threads Reference where number of threads should be stored.
     * @param xmlFilePath Reference where file path to CommonRoad xml should be stored.
    */
    int readCommandLineValues(int argc, char *const *argv, int &num_threads, std::string &xmlFilePath);
}

#endif //ENV_MODEL_COMMAND_LINE_INPUT_H

