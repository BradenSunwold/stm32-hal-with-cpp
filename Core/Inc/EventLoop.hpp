
#ifndef INC_EVENTLOOP_HPP_
#define INC_EVENTLOOP_HPP_

#include <stdint.h>
#include <stdio.h>

// Define func pointers for reading hw timers and timer overflows
typedef uint32_t (*ReadHwTimer)();
typedef uint32_t (*ReadOverflow)();

void EventLoopCpp();	// Cpp function to call into main event loop


#ifdef __cplusplus
extern "C"
{
#endif

	void EventLoopC();
	void InitializeInterface(ReadHwTimer readHwTimer, ReadOverflow readTimerOverflow);

#ifdef __cplusplus
}
#endif


#endif /* INC_EVENTLOOP_HPP_ */
