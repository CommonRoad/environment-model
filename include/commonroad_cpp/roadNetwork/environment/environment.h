#pragma once

class Environment {
  public:
    Environment();

    Environment(time_step_t timeStep,
                TimeOfDay timeOfDay,
                Weather weather,
                Underground underground);

  private:
    time_step_t timeStep{0};
    TimeOfDay timeOfDay{TimeOfDay::unknown};
    Weather weather{Weather::unknown};
    Underground underground{Underground::unknown};
};
