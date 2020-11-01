//
// Created by sebastian on 01.11.20.
//

#ifndef ENV_MODEL_STATE_H
#define ENV_MODEL_STATE_H


class State {
    private:
        double xPosition{0.0};
        double yPosition{0.0};
        double velocity{0.0};
        double acceleration{0.0};
        double lonPosition{0.0};
        double latPosition{0.0};
        double orientation{0.0};
};


#endif //ENV_MODEL_STATE_H
