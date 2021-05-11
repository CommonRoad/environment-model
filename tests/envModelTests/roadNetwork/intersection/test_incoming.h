//
// Created by Sebastian Maierhofer on 31.12.20.
//

#ifndef ENV_MODEL_TEST_INCOMING_H
#define ENV_MODEL_TEST_INCOMING_H

#include "commonroad_cpp/roadNetwork/intersection/incoming.h"
#include <gtest/gtest.h>

class IncomingTestInitialization {
  protected:
    std::unique_ptr<Incoming> incomingOne;
    std::unique_ptr<Incoming> incomingTwo;
    std::unique_ptr<Incoming> incomingThree;
    void setUpIncoming();
};

class IncomingTest : public IncomingTestInitialization, public testing::Test {
  private:
    void SetUp() override;
};

#endif // ENV_MODEL_TEST_INCOMING_H
