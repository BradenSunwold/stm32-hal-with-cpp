
#include "CppLedBlink.hpp"

// Constructor
CppLedBlink::CppLedBlink()
{
	while(1)
	{
		for(int i = 0; i < 1000000; i++);		// arbitrary delay
		mPinout.mTestLed.Toggle();
	}
}
