#include "test_high_computation_time_test_predicate.h"
#include <chrono>
using namespace std::chrono;

TEST_F(TestHighComputationTimeTestPredicate, time) { EXPECT_TRUE(pred.booleanEvaluation(0, nullptr, nullptr)); }

TEST_F(TestHighComputationTimeTestPredicate, ExecutionTime) {
    auto start = high_resolution_clock::now();
    pred.booleanEvaluation(0, nullptr, nullptr);
    auto stop = high_resolution_clock::now();

    auto duration = duration_cast<milliseconds>(stop - start);

    ASSERT_GE(duration.count(), 100);
}
