#pragma once

#include <commonroad_cpp/interfaces/commonroad/input_utils.h>
#include <gtest/gtest.h>

class InterfacesTest : public testing::Test {

  protected:
    /**
     * Loads two scenarios from same file name with xml and protobuf format.
     *
     * @param name File name
     * @return Xml and protobuf scenario
     */
    static std::tuple<Scenario, Scenario> loadXmlAndPbScenarios(const std::string &name);
};
