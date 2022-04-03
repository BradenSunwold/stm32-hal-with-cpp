
#ifndef CPPBLINKPINOUT_HPP_
#define CPPBLINKPINOUT_HPP_

#include "GpioPin.hpp"
#include "main.h"

class CppBlinkPinout
{
public:
	CppBlinkPinout();

	// Define all needed Gpio pins as member variables
	GpioPin mTestLed;

};


#endif /* CPPBLINKPINOUT_HPP_ */
