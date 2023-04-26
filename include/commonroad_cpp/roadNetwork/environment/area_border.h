#pragma once

class AreaBorder{
  public:
    /**
     * Default Constructor
     */
    AreaBorder();

    /**
     * TODO
     * @param areaBorderID
     * @param borderPoints
     * @param adjacentID
     * @param lineMarking
     */
    AreaBorder(size_t areaBorderID, std::vector<vertex> &borderPoints, std::optional<int> &adjacentID, LineMarking lineMarking);

  private:
    size_t areaBorderID{0};
    std::vector<vertex> borderPoints{};
    std::optional<int> adjacentID{};
    LineMarking lineMarking{LineMarking::unknown};
};
