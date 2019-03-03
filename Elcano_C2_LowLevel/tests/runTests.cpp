#include "pch.h"

#include "arduino.h"
#include "BrakesTest.h"
#include "SteeringControllerTest.h"
#include "ThrottleControllerTest.h"



int main(int argc, char **argv) {


	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

