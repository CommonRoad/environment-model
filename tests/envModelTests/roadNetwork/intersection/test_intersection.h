//
// Created by Sebastian Maierhofer on 31.12.20.
//

#ifndef ENV_MODEL_TEST_INTERSECTION_H
#define ENV_MODEL_TEST_INTERSECTION_H

#include "test_incoming.h"


class IntersectionTestInitialization : public IncomingTestInitialization {
protected:
    void setUpIntersection();
};

class IntersectionTest : public IntersectionTestInitialization, public testing::Test {
private:
    void SetUp() override;
};




#endif //ENV_MODEL_TEST_INTERSECTION_H
