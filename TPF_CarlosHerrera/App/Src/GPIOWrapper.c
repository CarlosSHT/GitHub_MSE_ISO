#include "GPIOWrapper.h"
#include "gpio.h"

void gpioSetLevel(uint16_t pin, uint32_t port, bool value)
{
    HAL_GPIO_WritePin((GPIO_TypeDef* )port, pin, !value);
}


bool gpioGetLevel(uint16_t pin, uint32_t port)
{
    return (HAL_GPIO_ReadPin((GPIO_TypeDef* )port, pin));
}
