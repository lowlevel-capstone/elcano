#pragma once
#include "arduino.h"
#include "PID_v1.h"

#include "../../ThrottleController.cpp"

class ThrottleControllerTest : public::testing::Test {
protected:
	void SetUp() override {
		t.initialize();
	}
	ThrottleController t;
};

TEST_F(ThrottleControllerTest, init) {
	EXPECT_EQ(dPins[SelectAB].mode, OUTPUT);
	EXPECT_EQ(dPins[SelectCD].mode, OUTPUT);
	EXPECT_EQ(Interrupt.intNum, digitalPinToInterrupt(IRPT_WHEEL));
}

TEST_F(ThrottleControllerTest, stop_str100) {
	//throttle.stop should stop the throttle quickly
	//TODO:
	//fix spi.transfer
}

TEST_F(ThrottleControllerTest, stop) {

}

TEST_F(ThrottleControllerTest, engage) {

}

TEST_F(ThrottleControllerTest, getSpeed) {

}

TEST_F(ThrottleControllerTest, setSpeed) {

}

TEST_F(ThrottleControllerTest, setdSpeed) {

}

TEST_F(ThrottleControllerTest, update) {

}

TEST_F(ThrottleControllerTest, tick) {

}

