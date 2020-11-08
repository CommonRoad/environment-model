//
// Created by Sebastian Maierhofer on 01.11.20.
//

#ifndef ENV_MODEL_STATE_H
#define ENV_MODEL_STATE_H


class State {
public:
    [[nodiscard]] double getXPosition() const;
    [[nodiscard]] double getYPosition() const;
    [[nodiscard]] double getVelocity() const;
    [[nodiscard]] double getAcceleration() const;
    [[nodiscard]] double getLonPosition() const;
    [[nodiscard]] double getLatPosition() const;
    [[nodiscard]] double getOrientation() const;
    [[nodiscard]] int getTimeStep() const;

    void setXPosition(double xPosition);
    void setYPosition(double yPosition);
    void setVelocity(double velocity);
    void setAcceleration(double acceleration);
    void setLonPosition(double lonPosition);
    void setLatPosition(double latPosition);
    void setOrientation(double orientation);
    void setTimeStep(int timeStep);

private:
        double xPosition{0.0};
        double yPosition{0.0};
        double velocity{0.0};
        double acceleration{0.0};
        double lonPosition{0.0};
        double latPosition{0.0};
        double orientation{0.0};
        int timeStep{0};
};


#endif //ENV_MODEL_STATE_H