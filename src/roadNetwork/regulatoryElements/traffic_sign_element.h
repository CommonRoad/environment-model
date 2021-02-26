//
// Created by Sebastian Maierhofer on 31.10.20.
//

#ifndef ENV_MODEL_TRAFFIC_SIGN_ELEMENT_H
#define ENV_MODEL_TRAFFIC_SIGN_ELEMENT_H

#include "../../auxiliaryDefs/structs.h"

/**
 * Class representing a CommonRoad traffic sign element.
 */
class TrafficSignElement {
public:
    /**
     * Constructor of traffic sign element.
     *
     * @param id ID of traffic sign element. Note this ID corresponds to the official national traffic sign ID of a country.
     */
    explicit TrafficSignElement(std::string  id);

    /**
     * Copy constructor of traffic sign element.
     *
     */
    TrafficSignElement(const TrafficSignElement &) = default;

    /**
     * Getter for ID of traffic sign element.
     *
     * @return ID of traffic sign element.
     */
    [[nodiscard]] std::string getId() const;

    /**
     * Getter for additional values of traffic sign element.
     *
     * @return Additional value.
     */
    [[nodiscard]] std::vector<std::string> getAdditionalValues() const;

    /**
     * Adds an additional value to a traffic sign element.
     *
     * @return Additional value.
     */
    void addAdditionalValue(const std::string &value);

    /**
     * Adds a list of additional values to a traffic sign element.
     *
     * @return Additional value.
     */
    void setAdditionalValues(const std::vector<std::string>& values);

    /**
     *
     * @param trafficSignName CommonRoad name of traffic sign.
     * @param country Country for which traffic sign number should be extracted.
     * @return String corresponding to traffic sign ID of country.
     */
    static std::string mapTrafficSignNameToCountryID(const std::string& trafficSignName, SupportedTrafficSignCountry country);

    /**
     * Converts German traffic sign ID to string.
     *
     * @param signId Traffic sign ID from corresponding enum.
     * @return String which corresponds to traffic sign ID.
     */
    static std::string convertGermanTrafficSignIdToString(TrafficSignIDGermany signId);

private:
    std::string id;                                  //**< official national traffic sign ID of a country */
    std::vector<std::string> additionalValues;       //**< list of additional values classifying traffic sign, e.g., velocity, weight, time */

};

#endif //ENV_MODEL_TRAFFIC_SIGN_ELEMENT_H
