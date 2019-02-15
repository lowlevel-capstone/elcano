#pragma once
#include "../../Brakes.cpp"
#include "arduino.h"

class BrakesTest : public ::testing::Test {
protected:
	void SetUp() override {
		returnMillis = 0;
		b.initialize();
		if (RELAYInversion) {
			pOn = LOW;
			pOff = HIGH;
		}
		else {
			pOn = HIGH;
			pOff = LOW;
		}
	}
	Brakes b;
	pVal pOn;
	pVal pOff;
};

TEST_F(BrakesTest, init) {

	EXPECT_EQ(dPins[BrakeOnPin].mode, OUTPUT);
	EXPECT_EQ(dPins[BrakeVoltPin].mode, OUTPUT);

	EXPECT_EQ(dPins[BrakeOnPin].val, pOff);
	EXPECT_EQ(dPins[BrakeVoltPin].val, pOff);
}

TEST_F(BrakesTest, release) {
	b.Release();
	EXPECT_EQ(dPins[BrakeOnPin].val, pOff);
	EXPECT_EQ(dPins[BrakeVoltPin].val, pOff);

}

TEST_F(BrakesTest, stop) {
	returnMillis = 0.1;
	b.Stop();
	EXPECT_EQ(dPins[BrakeVoltPin].val, pOn);
	EXPECT_EQ(dPins[BrakeOnPin].val, pOn);
}

TEST_F(BrakesTest, check_brakes_0ff) {
	returnMillis = MaxHi_ms - 20;
	b.Check();
	EXPECT_EQ(dPins[BrakeOnPin].val, pOff);
	EXPECT_EQ(dPins[BrakeVoltPin].val, pOff);
}

TEST_F(BrakesTest, check_brakes_on_lessTime) {
	b.Stop();
	returnMillis = MaxHi_ms - 20;

	b.Check();
	EXPECT_EQ(dPins[BrakeOnPin].val, pOn);
	EXPECT_EQ(dPins[BrakeVoltPin].val, pOn);
}

TEST_F(BrakesTest, check_brakes_on_moreTime) {
	b.Stop();
	returnMillis = MaxHi_ms + 2;

	b.Check();
	EXPECT_EQ(dPins[BrakeOnPin].val, pOn);
	EXPECT_EQ(dPins[BrakeVoltPin].val, pOff);
}