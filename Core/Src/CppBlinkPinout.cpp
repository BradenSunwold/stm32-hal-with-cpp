
#include "CppBlinkPinout.hpp"


CppBlinkPinout::CppBlinkPinout()
	:
	// Wrap all GPIO HAL pins / ports to project from main.h
	mTestLed(LED2_Pin, LED2_GPIO_Port)
{
}

