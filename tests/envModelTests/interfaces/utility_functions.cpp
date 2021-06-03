//
// Created by Sebastian Maierhofer on 21.02.21.
//

#include "utility_functions.h"
#include <boost/filesystem.hpp>
#include <sys/stat.h>

std::string TestUtils::getTestScenarioDirectory() {
    std::string curDir{get_current_dir_name()};
    std::string srcDir{curDir + "/tests/scenarios"};
    if (!existsDirectory(srcDir)) {
        srcDir = curDir + "/../tests/scenarios";
        existsDirectory(srcDir);
    }
    return srcDir;
}

bool TestUtils::existsDirectory(const std::string &path) {
    struct stat info {};
    if (stat(path.c_str(), &info) != 0)
        return false;
    return (info.st_mode & S_IFDIR) != 0u;
}