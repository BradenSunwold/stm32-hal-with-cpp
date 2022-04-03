
#ifndef GPIOPIN_HPP_
#define GPIOPIN_HPP_

#include "stm32l4xx_hal.h"

class GpioPin
{
public:
	GpioPin(uint16_t pin, GPIO_TypeDef* port);

	bool Read();
	void Set();
	void Clear();
	void Toggle();

private:
	uint16_t mGpioPin;
	GPIO_TypeDef* mGpioPort;

};


#endif /* GPIOPIN_HPP_ */
