//
// Created by Sebastian Maierhofer on 01.11.20.
//

#ifndef ENV_MODEL_STATE_H
#define ENV_MODEL_STATE_H


class State {
public:
    [[nodiscard]] double getXPosition() const;

    void setXPosition(double xPosition);

    [[nodiscard]] double getYPosition() const;

    void setYPosition(double yPosition);

    [[nodiscard]] double getVelocity() const;

    void setVelocity(double velocity);

    [[nodiscard]] double getAcceleration() const;

    void setAcceleration(double acceleration);

    [[nodiscard]] double getLonPosition() const;

    void setLonPosition(double lonPosition);

    [[nodiscard]] double getLatPosition() const;

    void setLatPosition(double latPosition);

    [[nodiscard]] double getOrientation() const;

    void setOrientation(double orientation);

private:
        double xPosition{0.0};
        double yPosition{0.0};
        double velocity{0.0};
        double acceleration{0.0};
        double lonPosition{0.0};
        double latPosition{0.0};
        double orientation{0.0};
        int timeStep{0};
public:
    int getTimeStep() const;

    void setTimeStep(int timeStep);
};


#endif //ENV_MODEL_STATE_H
