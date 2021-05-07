//
// Created by Sebastian Maierhofer on 21.02.21.
//

#include "utility_functions.h"
#include <boost/filesystem.hpp>
#include <sys/stat.h>

std::string TestUtils::getTestScenarioDirectory() {
    std::string curDir{get_current_dir_name()};
    std::string srcDir{curDir + "/tests/testScenarios"};
    if (!existsDirectory(srcDir)) {
        srcDir = curDir + "/../tests/testScenarios";
        existsDirectory(srcDir);
    }
    return srcDir;
}

bool TestUtils::existsDirectory(const std::string &path) {
    struct stat info {};
    if (stat(path.c_str(), &info) != 0)
        return false;
    if (info.st_mode & S_IFDIR)
        return true;
    else
        return false;
}