//
// Created by Sebastian Maierhofer on 21.02.21.
//

#ifndef ENV_MODEL_UTILITY_FUNCTIONS_H
#define ENV_MODEL_UTILITY_FUNCTIONS_H

#include <string>

namespace TestUtils {

bool existsDirectory(const std::string &path);

std::string getTestScenarioDirectory();
} // namespace TestUtils

#endif // ENV_MODEL_UTILITY_FUNCTIONS_H
