//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "utility_functions.h"
#include <boost/filesystem.hpp>
#include <filesystem>
#include <fstream>
#include <sys/stat.h>
#include <tuple>

std::string TestUtils::getTestScenarioDirectory() {
    std::string curDir{std::filesystem::current_path()};
    std::string srcDir{curDir + "/tests/scenarios"};
    if (!existsDirectory(srcDir) && existsDirectory(curDir + "/../tests/scenarios"))
        srcDir = curDir + "/../tests/scenarios";
    else if (!existsDirectory(srcDir) && existsDirectory(curDir + "/../../tests/scenarios"))
        srcDir = curDir + "/../../tests/scenarios";
    return srcDir;
}

bool TestUtils::existsDirectory(const std::string &path) {
    struct stat info {};
    if (stat(path.c_str(), &info) != 0)
        return false;
    return (info.st_mode & S_IFDIR) != 0u;
}

void TestUtils::copyAndReplaceContentInFile(const std::string &orgFileName, const std::string &newFileName,
                                            const std::vector<std::tuple<std::string, std::string>> &stringList) {
    std::filesystem::remove(newFileName);
    std::ifstream filein{orgFileName};
    std::ofstream fileout{newFileName};
    std::string line;
    while (getline(filein, line)) {
        for (const auto &elem : stringList) {
            std::string strReplace{std::get<0>(elem)};
            std::string strNew{std::get<1>(elem)};
            size_t len{strReplace.length()};
            while (true) {
                size_t pos = line.find(strReplace);
                if (pos != std::string::npos)
                    line.replace(pos, len, strNew);
                else
                    break;
            }
        }
        fileout << line << '\n';
    }
}