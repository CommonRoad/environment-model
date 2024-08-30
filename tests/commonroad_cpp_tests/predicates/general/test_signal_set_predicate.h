#pragma oncw
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/general/signal_set_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class SignalSetPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    std::shared_ptr<World> world;
    SignalSetPredicate pred;

  private:
    void SetUp() override;
};
