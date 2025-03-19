#include "utility_functions.h"
#include <boost/filesystem.hpp>
#include <filesystem>
#include <fstream>
#include <sys/stat.h>
#include <tuple>

std::string TestUtils::getTestScenarioDirectory() {
    auto curDir{std::filesystem::current_path()};
    auto parent{curDir.parent_path()};

    for (const auto &candidate : {curDir, parent, parent.parent_path()}) {
        auto candidateScenarioDir{candidate / "tests" / "scenarios"};
        if (std::filesystem::is_directory(candidateScenarioDir)) {
            return candidateScenarioDir.string();
        }
    }

    throw std::runtime_error{"could not find test scenario directory"};
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
