#include "test_low_computation_time_test_predicate.h"
#include <chrono>
using namespace std::chrono;

TEST_F(TestLowComputationTimeTestPredicate, time) { EXPECT_TRUE(pred.booleanEvaluation(0, nullptr, nullptr)); }

TEST_F(TestLowComputationTimeTestPredicate, ExecutionTime) {
    auto start = high_resolution_clock::now();
    pred.booleanEvaluation(0, nullptr, nullptr);
    auto stop = high_resolution_clock::now();

    auto duration = duration_cast<seconds>(stop - start);

    ASSERT_LE(duration.count(), 0.010);
}
