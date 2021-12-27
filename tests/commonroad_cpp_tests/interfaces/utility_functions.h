//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "vector"
#include <string>

namespace TestUtils {

bool existsDirectory(const std::string &path);

std::string getTestScenarioDirectory();

void copyAndReplaceContentInFile(const std::string &orgFileName, const std::string &newFileName,
                                 const std::vector<std::tuple<std::string, std::string>> &stringList);

} // namespace TestUtils
