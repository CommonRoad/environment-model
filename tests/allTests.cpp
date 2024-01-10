#include <gtest/gtest.h>
#include <iostream>

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    // set the gtest death test style to threadsafe
    testing::FLAGS_gtest_death_test_style = "threadsafe";
    // testing::GTEST_FLAG(filter) = "CrossingOutgoingWithDirPredicateTest.BothRight";
    int result = RUN_ALL_TESTS();
#ifdef CODE_COVERAGE
    if (result != 0) {
        std::cout << "WARNING: One or more tests have failed, but we indicate success anyway "
                     "by returning 0 because code coverage collection is enabled."
                  << std::endl;
    }
    return 0;
#else
    return result;
#endif
}
