
#include "GpioPin.hpp"

GpioPin::GpioPin(uint16_t pin, GPIO_TypeDef* port)
	:
	mGpioPin(pin),
	mGpioPort(port)
{
}

bool GpioPin::Read()
{
	return HAL_GPIO_ReadPin(mGpioPort, mGpioPin);
}

void GpioPin::Set()
{
	HAL_GPIO_WritePin(mGpioPort, mGpioPin, GPIO_PIN_SET);
}

void GpioPin::Clear()
{
	HAL_GPIO_WritePin(mGpioPort, mGpioPin, GPIO_PIN_RESET);
}

void GpioPin::Toggle()
{
	HAL_GPIO_TogglePin(mGpioPort, mGpioPin);
}


