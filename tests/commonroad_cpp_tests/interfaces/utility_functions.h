//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "vector"
#include <string>

/**
 * Collection of utility functions for test case execution.
 */
namespace TestUtils {

/**
 * Checks whether a given directory exists.
 *
 * @param path Directory path.
 * @return Boolean value indicating that directory path exists.
 */
bool existsDirectory(const std::string &path);

/**
 * Searches for path to directory where test scenarios are located.
 *
 * @return Path.
 */
std::string getTestScenarioDirectory();

/**
 * Copies a text-based file and replaces specific content in file.
 *
 * @param orgFileName Path to the file which should be copied.
 * @param newFileName Path to the new file.
 * @param stringList List of tuples containing string which should be replaced and new string.
 */
void copyAndReplaceContentInFile(const std::string &orgFileName, const std::string &newFileName,
                                 const std::vector<std::tuple<std::string, std::string>> &stringList);

} // namespace TestUtils
