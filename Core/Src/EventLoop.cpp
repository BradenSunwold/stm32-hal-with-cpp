
#include "EventLoop.hpp"
#include "CppLedBlink.hpp"		// Have to include any Cpp classes used in the EventLoop.cpp file

// Main Cpp event loop to run app
void EventLoopCpp()
{
	CppLedBlink testing;
}

// All C calls from main.c below
extern "C"
{
	void EventLoopC()
	{
		EventLoopCpp();
	}

	void InitializeInterface(ReadHwTimer readHwTimer, ReadOverflow readTimerOverflow)
	{
		/* Can now link these function pointers to any C++ class for use.
		 * For example, Could pass them into a static global system time
		 * class where they would be used to calculate the ongoing system
		 * time from system startup. */
	}
}
