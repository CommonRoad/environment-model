#pragma once

#include "vector"
#include <string>

/**
 * Collection of utility functions for test case execution.
 */
namespace TestUtils {

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
