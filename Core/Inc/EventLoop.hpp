
#ifndef INC_EVENTLOOP_HPP_
#define INC_EVENTLOOP_HPP_

#include <stdint.h>
#include <stdio.h>

void EventLoopCpp();	// Cpp function to call into main event loop


#ifdef __cplusplus
extern "C"
{
#endif

	void EventLoopC();

#ifdef __cplusplus
}
#endif


#endif /* INC_EVENTLOOP_HPP_ */
