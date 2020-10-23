//
// Created by sebastian on 23.10.20.
//

#ifndef ENVIRONMENT_MODEL_LANELET_H
#define ENVIRONMENT_MODEL_LANELET_H


class Lanelet {
    public:
        lanelet() { id = 0; }
        lanelet(const lanelet &) = default;            // copy constructor
        lanelet &operator=(const lanelet &) = default; // copy assignment
        lanelet(lanelet &&) = default;                 // move constructor
        lanelet &operator=(lanelet &&) = default;      // move assignment
        virtual ~lanelet() = default;                  // virtual destructor

        /*
         * setter functions
         */
        void addLeftVertice(const vertice left);
        void addRightVertice(const vertice right);
        void addCenterVertice(const vertice center);

        void setLeftBorderVertices(const std::vector<vertice> &leftBorderVertices) override;
        void setRightBorderVertices(const std::vector<vertice> &rightBorderVertices) override;
        void setCenterVertices(const std::vector<vertice> &center) override;

        // Takes rvalue and moves the data
        void moveLeftBorder(std::vector<vertice> &&leftBorderVertices);
        void moveRightBorder(std::vector<vertice> &&rightBorderVertices);
        void moveCenterVertices(std::vector<vertice> &&center);
        void createCenterVertices();

        /*
         * getter functions
         */
        std::vector<vertice> getLeftBorderVerticesDirect() const;
        std::vector<vertice> getRightBorderVerticesDirect() const;
        std::vector<vertice> getCenterVerticesDirect() const;

        const std::vector<vertice> &getCenterVertices() const override;
        const std::vector<vertice> &getLeftBorderVertices() const override;
        const std::vector<vertice> &getRightBorderVertices() const override;

    private:
        std::vector<vertice> leftBorder;  // vertices of left border
        std::vector<vertice> rightBorder; // vertices of right border
};


#endif //ENVIRONMENT_MODEL_LANELET_H
