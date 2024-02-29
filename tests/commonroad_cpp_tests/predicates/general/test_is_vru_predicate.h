#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/general/is_vru_predicate.h>
#include <gtest/gtest.h>

class TestIsVruPredicate : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    IsVruPredicate pred;

  private:
    void SetUp() override;
};
