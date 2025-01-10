#include "test_traffic_sign.h"
#include "commonroad_cpp/auxiliaryDefs/regulatory_elements.h"

void TrafficSignTest::SetUp() {
    std::vector<std::string> trafficSignElementOneValues{"20"};
    auto signElement1 = std::vector<std::shared_ptr<TrafficSignElement>>{
        std::make_shared<TrafficSignElement>(TrafficSignTypes::MAX_SPEED, trafficSignElementOneValues)};

    std::vector<std::string> trafficSignElementTwoValues{"10"};
    auto signElement2 = std::vector<std::shared_ptr<TrafficSignElement>>{
        std::make_shared<TrafficSignElement>(TrafficSignTypes::MIN_SPEED, trafficSignElementTwoValues)};

    size_t signId1{1};
    vertex pos1{1, 2};
    sign1 = std::make_shared<TrafficSign>(signId1, signElement1, pos1, false);

    size_t signId2{2};
    vertex pos2{3, 4};
    sign2 = std::make_shared<TrafficSign>();
    sign2->setId(signId2);
    sign2->setPosition(pos2);
    sign2->setTrafficSignElement(signElement2);
    sign2->addTrafficSignElement(signElement1.at(0));
    sign2->setVirtualElement(true);
}

TEST_F(TrafficSignTest, InitializationComplete) {
    EXPECT_EQ(sign1->getId(), 1);
    EXPECT_EQ(sign1->getPosition().x, 1);
    EXPECT_EQ(sign1->getPosition().y, 2);
    EXPECT_EQ(sign1->isVirtualElement(), false);
    EXPECT_EQ(sign1->getTrafficSignElements().size(), 1);
    EXPECT_EQ(sign1->getTrafficSignElements().at(0)->getTrafficSignType(), TrafficSignTypes::MAX_SPEED);

    EXPECT_EQ(sign2->getId(), 2);
    EXPECT_EQ(sign2->getPosition().x, 3);
    EXPECT_EQ(sign2->getPosition().y, 4);
    EXPECT_EQ(sign2->isVirtualElement(), true);
    EXPECT_EQ(sign2->getTrafficSignElements().size(), 2);
    EXPECT_EQ(sign2->getTrafficSignElements().at(0)->getTrafficSignType(), TrafficSignTypes::MIN_SPEED);
    EXPECT_EQ(sign2->getTrafficSignElements().at(1)->getTrafficSignType(), TrafficSignTypes::MAX_SPEED);
}

TEST_F(TrafficSignTest, GetTrafficSignElementsOfType) {
    EXPECT_EQ(sign1->getTrafficSignElementsOfType(TrafficSignTypes::MAX_SPEED).size(), 1);
    EXPECT_EQ(sign1->getTrafficSignElementsOfType(TrafficSignTypes::WARNING_CONSTRUCTION_SITE).size(), 0);

    EXPECT_EQ(sign2->getTrafficSignElementsOfType(TrafficSignTypes::MAX_SPEED).size(), 1);
    EXPECT_EQ(sign2->getTrafficSignElementsOfType(TrafficSignTypes::MIN_SPEED).size(), 1);
}
