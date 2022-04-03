
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
}
