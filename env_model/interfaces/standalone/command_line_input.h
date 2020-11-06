//
// Created by sebastian on 02.11.20.
//

#ifndef ENV_MODEL_COMMAND_LINE_INPUT_H
#define ENV_MODEL_COMMAND_LINE_INPUT_H

#include <iostream>

namespace CommandLine{
    int readCommandLineValues(int argc, char *const *argv, int &num_threads, std::string &xmlFilePath);
}


#endif //ENV_MODEL_COMMAND_LINE_INPUT_H
