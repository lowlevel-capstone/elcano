#pragma once
#include "arduino.h"
#include "PID_v1.h"

#include "../../SteeringController.cpp"


class SteeringControllerTest : public ::testing::Test {
protected:
	void SetUp() override {
		lft_angle = MIN_TURN;
		returnMillis = 0;
		s.initialize(lft_angle);

		if (RELAYInversion) {
			pOn = LOW;
			pOff = HIGH;
		}
		else {
			pOn = HIGH;
			pOff = LOW;
		}
	}
	SteeringController s;
	int32_t lft_angle;
	pVal pOn;
	pVal pOff;

};

TEST_F(SteeringControllerTest, init) {
	EXPECT_EQ(dPins[STEER_ON].mode, OUTPUT);
	EXPECT_EQ(dPins[STEER_OUT_PIN].angle, lft_angle);
	EXPECT_EQ(dPins[STEER_ON].val, pOn);
}

TEST_F(SteeringControllerTest, engage_OVER_MAX) {
	int32_t turn = MAX_TURN + 5;
	s.engageSteering(turn);
	EXPECT_EQ(dPins[STEER_OUT_PIN].angle, MAX_TURN);
}

TEST_F(SteeringControllerTest, engage_UNDER_MIN) {
	int32_t turn = MIN_TURN - 5;
	s.engageSteering(turn);
	EXPECT_EQ(dPins[STEER_OUT_PIN].angle, MIN_TURN);
}

TEST_F(SteeringControllerTest, engage_overflow) {
	//max uint32 value, can't be held by int32_t 
	uint32_t turn = 4294967295;
	EXPECT_THROW(s.engageSteering(turn), std::out_of_range);
}

TEST_F(SteeringControllerTest, using_Pid) {
	int32_t cturn = CENTER_TURN;
	int32_t dturn = MAX_TURN;
	int x = 0;
	returnMillis = 0;
	s.setDesiredTurn(cturn);
	returnMillis = 100;
	s.updateAnglePID(cturn);

	s.setDesiredTurn(dturn);

	while (dPins[STEER_OUT_PIN].angle != cturn) {
		s.updateAnglePID(dPins[STEER_OUT_PIN].angle);

		returnMillis += 100;
		x++;
		std::cout << returnMillis << ": " << dPins[STEER_OUT_PIN].angle << std::endl;
		EXPECT_LT(x, 50);
	}
	ASSERT_EQ(dPins[STEER_OUT_PIN].angle, cturn);

	x = 0;
	s.setDesiredTurn(dturn);
	while (dPins[STEER_OUT_PIN].angle != dturn) {
		s.updateAnglePID(dPins[STEER_OUT_PIN].angle);

		returnMillis += 100;
		x++;
		std::cout << returnMillis << ": " << dPins[STEER_OUT_PIN].angle << std::endl;
		EXPECT_LT(x, 200);
	}


	dturn = MIN_TURN;
	x = 0;
	s.setDesiredTurn(dturn);
	while (dPins[STEER_OUT_PIN].angle != dturn) {
		s.updateAnglePID(dPins[STEER_OUT_PIN].angle);

		returnMillis += 100;
		x++;
		std::cout << returnMillis << ": " << dPins[STEER_OUT_PIN].angle << std::endl;
		EXPECT_LT(x, 200);
	}
}
