/*
 * Copyright (C) 2022 Jesús Bautista Villar
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */
 
#ifndef CONFIG_MATEK_F405_WMN_H
#define CONFIG_MATEK_F405_WMN_H

#define CONFIG_MATEK_F405_WMN

/* The Matek F405 WMN autopilot has a 8MHz external clock and 168MHz internal. */
#define EXT_CLK 8000000
#define AHB_CLK 168000000

// Onboard LEDs
/* Blue (SYS) */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOA
#define LED_1_GPIO_PIN GPIO14
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/* Green (GPS) */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOA
#define LED_2_GPIO_PIN GPIO13
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP ((void)0)

// Beeper
#define BEEPER_GPIO GPIOB
#define BEEPER_GPIO_PIN GPIO9
#define BEEPER_GPIO_ON gpio_clear
#define BEEPER_GPIO_OFF gpio_set
#define BEEPER_AFIO_REMAP ((void)0)

// Default actuators driver
#define DEFAULT_ACTUATORS "modules/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

/*** SPI1 IMU (ICM42688P) & OSD ************************************************/
#define SPI1_GPIO_AF GPIO_AF5

#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5
#define SPI1_GPIO_PORT_MISO GPIOB
#define SPI1_GPIO_MISO GPIO4
#define SPI1_GPIO_PORT_MOSI GPIOA
#define SPI1_GPIO_MOSI GPIO7
#define SPI1_GPIO_PORT_NSS GPIOC
#define SPI1_GPIO_NSS GPIO14

#define SPI_SELECT_SLAVE0_PORT GPIOC
#define SPI_SELECT_SLAVE0_PIN GPIO14

/*** SPI2 Flash ****************************************************************/
#define SPI2_GPIO_AF GPIO_AF5

#define SPI2_GPIO_PORT_SCK GPIOB
#define SPI2_GPIO_SCK GPIO13
#define SPI2_GPIO_PORT_MISO GPIOC
#define SPI2_GPIO_MISO GPIO2
#define SPI2_GPIO_PORT_MOSI GPIOC
#define SPI2_GPIO_MOSI GPIO3
#define SPI2_GPIO_PORT_NSS GPIOC
#define SPI2_GPIO_NSS GPIO13

#define SPI_SELECT_SLAVE1_PORT GPIOC
#define SPI_SELECT_SLAVE1_PIN GPIO13

/*** I2C /Baro/Mag *************************************************************/
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO8
#define I2C1_GPIO_SDA GPIO7

#define I2C2_GPIO_PORT GPIOB
#define I2C2_GPIO_SCL GPIO10
#define I2C2_GPIO_SDA GPIO11

/*** UART **********************************************************************/
#define UART1_GPIO_AF GPIO_AF7
#define UART1_GPIO_PORT_TX GPIOA
#define UART1_GPIO_TX GPIO9
#define UART1_GPIO_PORT_RX GPIOA
#define UART1_GPIO_RX GPIO10

#define UART2_GPIO_AF GPIO_AF7
#define UART2_GPIO_PORT_TX GPIOA
#define UART2_GPIO_TX GPIO2
#define UART2_GPIO_PORT_RX GPIOA
#define UART2_GPIO_RX GPIO3

#define UART3_GPIO_AF GPIO_AF7
#define UART3_GPIO_PORT_TX GPIOC
#define UART3_GPIO_TX GPIO10
#define UART3_GPIO_PORT_RX GPIOC
#define UART3_GPIO_RX GPIO11

#define UART4_GPIO_AF GPIO_AF8
#define UART4_GPIO_PORT_TX GPIOA
#define UART4_GPIO_TX GPIO0
#define UART4_GPIO_PORT_RX GPIOA
#define UART4_GPIO_RX GPIO1

#define UART5_GPIO_AF GPIO_AF8
#define UART5_GPIO_PORT_TX GPIOC
#define UART5_GPIO_TX GPIO12
#define UART5_GPIO_PORT_RX GPIOD
#define UART5_GPIO_RX GPIO2

#define UART6_GPIO_AF GPIO_AF8
#define UART6_GPIO_PORT_TX GPIOC
#define UART6_GPIO_TX GPIO6
#define UART6_GPIO_PORT_RX GPIOC
#define UART6_GPIO_RX GPIO7

/*** ADC ***********************************************************************/

#define USE_AD_TIM2 1

#ifndef USE_ADC_1
#define USE_ADC_1 1
#endif

#ifndef USE_ADC_2
#define USE_ADC_2 1
#endif

#ifndef USE_ADC_3
#define USE_ADC_3 1
#endif

#ifndef USE_ADC_4
#define USE_ADC_4 1
#endif

// POWER SUPPLY VOLTAGE MEASUREMENT INPUT
#if USE_ADC_1
#define AD1_1_CHANNEL 1
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOC
#define ADC_1_GPIO_PIN GPIO0
#endif

#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_1
#endif

// CURRENT MEASUREMENT INPUT
#if USE_ADC_2
#define AD1_2_CHANNEL 2
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOC
#define ADC_2_GPIO_PIN GPIO1
#ifndef CURRENT_ADC_IN
#define CURRENT_ADC_IN ADC_2
#endif
#endif

#ifndef ADC_CHANNEL_CURRENT
#define ADC_CHANNEL_CURRENT ADC_2
#endif

// RSSI MEASUREMENT INPUT
#if USE_ADC_3
#define AD1_3_CHANNEL 3
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT GPIOB
#define ADC_3_GPIO_PIN GPIO0
#endif

// AIRSPEED
#if USE_ADC_4
#define AD1_4_CHANNEL 4
#define ADC_4 AD1_4
#define ADC_4_GPIO_PORT GPIOC
#define ADC_4_GPIO_PIN GPIO0
#endif

// Default ADC functions
#define ADCVREF 3.300  // System Voltage in V
#define CURRENT_METER_SCALE   0.150 // V/A
#define VSUPPLY_SCALE_DEFAULT 2.100 // V


// int32_t microvolts = ((uint32_t)adcGetChannel(ADC_CURRENT) * ADCVREF * 100) / 0xFFF * 10 - (int32_t)batteryMetersConfig()->current.offset * 100;
// return microvolts / batteryMetersConfig()->current.scale; // current in 0.01A steps
// return (uint64_t)adcGetChannel(ADC_BATTERY) * batteryMetersConfig()->voltage.scale * ADCVREF / (0xFFF * 1000);
#define DefaultVoltageOfAdc(adc) (adc * ADCVREF * VSUPPLY_SCALE_DEFAULT / 0xFFF) //TODO: Fix it
#define DefaultMilliAmpereOfAdc(adc) (adc * ADCVREF / CURRENT_METER_SCALE / 0xFFF) //TODO: Fix it


/*** PPM ***********************************************************************/
#define USE_PPM_TIM9        1
#define PPM_CHANNEL         TIM_IC2
#define PPM_TIMER_INPUT     TIM_IC_IN_TI2
#define PPM_IRQ             NVIC_TIM1_BRK_TIM9_IRQ

// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC2IE
#define PPM_CC_IF           TIM_SR_CC2IF
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO3
#define PPM_GPIO_AF         GPIO_AF3

/*** PWM ***********************************************************************/

// SERVO DEFINITIONS
#define PWM_USE_TIM1  1
#define PWM_USE_TIM2  1
#define PWM_USE_TIM4  0
#define PWM_USE_TIM8  1
#define PWM_USE_TIM12 0
#define PWM_USE_TIM13 0

#define USE_PWM1  1 
#define USE_PWM2  1 
#define USE_PWM3  1 
#define USE_PWM4  1 
#define USE_PWM5  1 
#define USE_PWM6  1
#define USE_PWM7  1
#define USE_PWM8  1
#define USE_PWM9  0 // actuators_pwm_arch doesn't support more than 8 servos
#define USE_PWM10 0
#define USE_PWM11 0

#if USE_PWM1
#define PWM_SERVO_1 0
#define PWM_SERVO_1_TIMER TIM8
#define PWM_SERVO_1_GPIO GPIOC
#define PWM_SERVO_1_PIN GPIO9
#define PWM_SERVO_1_AF GPIO_AF3
#define PWM_SERVO_1_OC TIM_OC2
#define PWM_SERVO_1_OC_BIT (1<<1)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#if USE_PWM2
#define PWM_SERVO_2 1
#define PWM_SERVO_2_TIMER TIM8
#define PWM_SERVO_2_GPIO GPIOC
#define PWM_SERVO_2_PIN GPIO8
#define PWM_SERVO_2_AF GPIO_AF3
#define PWM_SERVO_2_OC TIM_OC1
#define PWM_SERVO_2_OC_BIT (1<<0)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#if USE_PWM3
#define PWM_SERVO_3 2
#define PWM_SERVO_3_TIMER TIM1
#define PWM_SERVO_3_GPIO GPIOB
#define PWM_SERVO_3_PIN GPIO15
#define PWM_SERVO_3_AF GPIO_AF1
#define PWM_SERVO_3_OC TIM_OC2
#define PWM_SERVO_3_OC_BIT (1<<1)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

#if USE_PWM4
#define PWM_SERVO_4 3
#define PWM_SERVO_4_TIMER TIM1
#define PWM_SERVO_4_GPIO GPIOA
#define PWM_SERVO_4_PIN GPIO8
#define PWM_SERVO_4_AF GPIO_AF1
#define PWM_SERVO_4_OC TIM_OC1
#define PWM_SERVO_4_OC_BIT (1<<0)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

#if USE_PWM5
#define PWM_SERVO_5 4
#define PWM_SERVO_5_TIMER TIM2
#define PWM_SERVO_5_GPIO GPIOB
#define PWM_SERVO_5_PIN GPIO11
#define PWM_SERVO_5_AF GPIO_AF3
#define PWM_SERVO_5_OC TIM_OC4
#define PWM_SERVO_5_OC_BIT (1<<3)
#else
#define PWM_SERVO_5_OC_BIT 0
#endif

#if USE_PWM6
#define PWM_SERVO_6 5
#define PWM_SERVO_6_TIMER TIM2
#define PWM_SERVO_6_GPIO GPIOB
#define PWM_SERVO_6_PIN GPIO10
#define PWM_SERVO_6_AF GPIO_AF3
#define PWM_SERVO_6_OC TIM_OC3
#define PWM_SERVO_6_OC_BIT (1<<2)
#else
#define PWM_SERVO_6_OC_BIT 0
#endif

#if USE_PWM7
#define PWM_SERVO_7 6
#define PWM_SERVO_7_TIMER TIM2
#define PWM_SERVO_7_GPIO GPIOB
#define PWM_SERVO_7_PIN GPIO3
#define PWM_SERVO_7_AF GPIO_AF9
#define PWM_SERVO_7_OC TIM_OC2
#define PWM_SERVO_7_OC_BIT (1<<1)
#else
#define PWM_SERVO_7_OC_BIT 0
#endif

#if USE_PWM8
#define PWM_SERVO_8 7
#define PWM_SERVO_8_TIMER TIM2
#define PWM_SERVO_8_GPIO GPIOA
#define PWM_SERVO_8_PIN GPIO15
#define PWM_SERVO_8_AF GPIO_AF9
#define PWM_SERVO_8_OC TIM_OC1
#define PWM_SERVO_8_OC_BIT (1<<0)
#else
#define PWM_SERVO_8_OC_BIT 0
#endif

#if USE_PWM9
#define PWM_SERVO_9 8
#define PWM_SERVO_9_TIMER TIM12
#define PWM_SERVO_9_GPIO GPIOB
#define PWM_SERVO_9_PIN GPIO14
#define PWM_SERVO_9_AF GPIO_AF9
#define PWM_SERVO_9_OC TIM_OC1
#define PWM_SERVO_9_OC_BIT (1<<0)
#else
#define PWM_SERVO_9_OC_BIT 0
#endif

#if USE_PWM10
#define PWM_SERVO_10 9
#define PWM_SERVO_10_TIMER TIM13
#define PWM_SERVO_10_GPIO GPIOA
#define PWM_SERVO_10_PIN GPIO6
#define PWM_SERVO_10_AF GPIO_AF9
#define PWM_SERVO_10_OC TIM_OC1
#define PWM_SERVO_10_OC_BIT (1<<0)
#else
#define PWM_SERVO_10_OC_BIT 0
#endif

#if USE_PWM11
#define PWM_SERVO_11 10
#define PWM_SERVO_11_TIMER TIM4
#define PWM_SERVO_11_GPIO GPIOB
#define PWM_SERVO_11_PIN GPIO6
#define PWM_SERVO_11_AF GPIO_AF2
#define PWM_SERVO_11_OC TIM_OC1
#define PWM_SERVO_11_OC_BIT (1<<0)
#else
#define PWM_SERVO_11_OC_BIT 0
#endif

// servos 1-2 on TIM8
#define PWM_TIM8_CHAN_MASK (PWM_SERVO_1_OC_BIT | PWM_SERVO_2_OC_BIT)

// servos 3-4 on TIM1
#define PWM_TIM1_CHAN_MASK (PWM_SERVO_3_OC_BIT | PWM_SERVO_4_OC_BIT)

// servos 5-8 on TIM2
#define PWM_TIM2_CHAN_MASK (PWM_SERVO_5_OC_BIT | PWM_SERVO_6_OC_BIT | PWM_SERVO_7_OC_BIT | PWM_SERVO_8_OC_BIT)

// servo  9   on TIM12
#if USE_PWM9
#define PWM_TIM12_CHAN_MASK (PWM_SERVO_9_OC_BIT)
#endif

// servo  10  on TIM13
#if USE_PWM10
#define PWM_TIM13_CHAN_MASK (PWM_SERVO_10_OC_BIT)
#endif

// servo  11  on TIM4
#if USE_PWM11
#define PWM_TIM4_CHAN_MASK (PWM_SERVO_11_OC_BIT)
#endif


#endif // CONFIG_MATEK_F405_WMN_H
