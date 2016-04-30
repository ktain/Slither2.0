#ifndef LED_H
#define LED_H

#include "stm32f4xx.h"

// LEDs
#define LED1_ON    GPIO_SetBits(GPIOC, GPIO_Pin_2)
#define LED1_OFF   GPIO_ResetBits(GPIOC, GPIO_Pin_2)
#define LED2_ON    GPIO_SetBits(GPIOC, GPIO_Pin_3)
#define LED2_OFF   GPIO_ResetBits(GPIOC, GPIO_Pin_3)
#define LED3_ON    GPIO_SetBits(GPIOA, GPIO_Pin_5)
#define LED3_OFF   GPIO_ResetBits(GPIOA, GPIO_Pin_5)
#define LED4_ON    GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define LED4_OFF   GPIO_ResetBits(GPIOA, GPIO_Pin_4)

// Emitters
#define LEM_F_ON   GPIO_SetBits(GPIOD, GPIO_Pin_2)
#define LEM_F_OFF  GPIO_ResetBits(GPIOD, GPIO_Pin_2)
#define LEM_D_ON   GPIO_SetBits(GPIOC, GPIO_Pin_12)
#define LEM_D_OFF  GPIO_ResetBits(GPIOC, GPIO_Pin_12)
#define REM_D_ON   GPIO_SetBits(GPIOC, GPIO_Pin_11)
#define REM_D_OFF  GPIO_ResetBits(GPIOC, GPIO_Pin_11)
#define REM_F_ON   GPIO_SetBits(GPIOC, GPIO_Pin_10)
#define REM_F_OFF  GPIO_ResetBits(GPIOC, GPIO_Pin_10)

#define ALL_LED_OFF LED1_OFF; \
                    LED2_OFF; \
                    LED3_OFF; \
										LED4_OFF

										
										
#define ALL_LED_ON  LED1_ON; \
                    LED2_ON; \
										LED3_ON; \
										LED4_ON
										
#define ALL_EM_OFF  LEM_F_OFF; \
                    LEM_D_OFF; \
										REM_D_OFF; \
										REM_F_OFF

void LED_Configuration(void);

#endif /* LED_H */
