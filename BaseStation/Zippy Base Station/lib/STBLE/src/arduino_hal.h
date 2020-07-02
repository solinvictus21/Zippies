#include "Arduino.h"

#define GPIO_PIN_SET HIGH
#define GPIO_PIN_RESET LOW

#define BNRG_SPI_CS_PORT -1
#define BNRG_SPI_EXTI_PORT -1
#define BNRG_SPI_RESET_PORT -1

#define BNRG_SPI_CS_PIN 10
#define BNRG_SPI_EXTI_PIN 2
#define BNRG_SPI_RESET_PIN 9

#define HAL_Delay(x) delay(x)

#define HAL_GPIO_WritePin(x, y, z) digitalWrite(y,z)

#define HAL_GPIO_ReadPin(x,y) digitalRead(y)

#define HAL_GetTick millis

#define __disable_irq() 0

#define __set_PRIMASK(x) 0
#define __get_PRIMASK() 0
