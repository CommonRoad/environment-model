#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/general/is_of_type_predicate.h>
#include <gtest/gtest.h>

class TestIsOfTypePredicate : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    IsOfTypePredicate pred;

  private:
    void SetUp() override;
};
