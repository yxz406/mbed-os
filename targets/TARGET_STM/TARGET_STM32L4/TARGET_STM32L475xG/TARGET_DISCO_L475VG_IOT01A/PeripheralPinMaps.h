/* mbed Microcontroller Library
 *******************************************************************************
 * Copyright (c) 2018, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */

//==============================================================================
// Notes
//
// - The pins mentioned Px_y_ALTz are alternative possibilities which use other
//   HW peripheral instances. You can use them the same way as any other "normal"
//   pin (i.e. PwmOut pwm(PA_7_ALT0);). These pins are not displayed on the board
//   pinout image on mbed.org.
//
// - The pins which are connected to other components present on the board have
//   the comment "Connected to xxx". The pin function may not work properly in this
//   case. These pins may not be displayed on the board pinout image on mbed.org.
//   Please read the board reference manual and schematic for more information.
//
// - Warning: pins connected to the default STDIO_UART_TX and STDIO_UART_RX pins are commented
//   See https://os.mbed.com/teams/ST/wiki/STDIO for more information.
//
//==============================================================================

#ifndef MBED_PERIPHERALPINMAPS_H
#define MBED_PERIPHERALPINMAPS_H

#include "PinNamesTypes.h"
#include <mstd_cstddef>

//*** ADC ***

MSTD_CONSTEXPR_OBJ_11 PinMap PinMap_ADC[] = {
    {PA_0,       ADC_1, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 5, 0)}, // ADC1_IN5 // Connected to ARD_D1 [UART4_TX]
    {PA_0_ALT0,  ADC_2, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 5, 0)}, // ADC2_IN5 // Connected to ARD_D1 [UART4_TX]
    {PA_1,       ADC_1, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 6, 0)}, // ADC1_IN6 // Connected to ARD_D0 [UART4_RX]
    {PA_1_ALT0,  ADC_2, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 6, 0)}, // ADC2_IN6 // Connected to ARD_D0 [UART4_RX]
    {PA_2,       ADC_1, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 7, 0)}, // ADC1_IN7 // Connected to ARD_D10 [SPI_SSN]
    {PA_2_ALT0,  ADC_2, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 7, 0)}, // ADC2_IN7 // Connected to ARD_D10 [SPI_SSN]
    {PA_3,       ADC_1, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 8, 0)}, // ADC1_IN8 // Connected to ARD_D4
    {PA_3_ALT0,  ADC_2, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 8, 0)}, // ADC2_IN8 // Connected to ARD_D4
    {PA_4,       ADC_1, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 9, 0)}, // ADC1_IN9 // Connected to ARD_D7
    {PA_4_ALT0,  ADC_2, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 9, 0)}, // ADC2_IN9 // Connected to ARD_D7
    {PA_5,       ADC_1, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 10, 0)}, // ADC1_IN10 // Connected to ARD_D13 [SPI1_SCK]
    {PA_5_ALT0,  ADC_2, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 10, 0)}, // ADC2_IN10 // Connected to ARD_D13 [SPI1_SCK]
    {PA_6,       ADC_1, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 11, 0)}, // ADC1_IN11 // Connected to ARD_D12 [SPI1_MISO]
    {PA_6_ALT0,  ADC_2, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 11, 0)}, // ADC2_IN11 // Connected to ARD_D12 [SPI1_MISO]
    {PA_7,       ADC_1, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 12, 0)}, // ADC1_IN12 // Connected to ARD_D11 [SPI1_MOSI]
    {PA_7_ALT0,  ADC_2, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 12, 0)}, // ADC2_IN12 // Connected to ARD_D11 [SPI1_MOSI]
    {PB_0,       ADC_1, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 15, 0)}, // ADC1_IN15 // Connected to ARD_D3 [INT_EXT10]
    {PB_0_ALT0,  ADC_2, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 15, 0)}, // ADC2_IN15 // Connected to ARD_D3 [INT_EXT10]
    {PB_1,       ADC_1, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 16, 0)}, // ADC1_IN16 // Connected to ARD_D6 [ADC1_IN6]
    {PB_1_ALT0,  ADC_2, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 16, 0)}, // ADC2_IN16 // Connected to ARD_D6 [ADC1_IN6]
    {PC_0,       ADC_1, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 1, 0)}, // ADC1_IN1 // Connected to ARD_A5 [ADC]
    {PC_0_ALT0,  ADC_2, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 1, 0)}, // ADC2_IN1 // Connected to ARD_A5 [ADC]
    {PC_0_ALT1,  ADC_3, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 1, 0)}, // ADC3_IN1 // Connected to ARD_A5 [ADC]
    {PC_1,       ADC_1, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 2, 0)}, // ADC1_IN2 // Connected to ARD_A4 [ADC]
    {PC_1_ALT0,  ADC_2, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 2, 0)}, // ADC2_IN2 // Connected to ARD_A4 [ADC]
    {PC_1_ALT1,  ADC_3, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 2, 0)}, // ADC3_IN2 // Connected to ARD_A4 [ADC]
    {PC_2,       ADC_1, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 3, 0)}, // ADC1_IN3 // Connected to ARD_A3 [ADC]
    {PC_2_ALT0,  ADC_2, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 3, 0)}, // ADC2_IN3 // Connected to ARD_A3 [ADC]
    {PC_2_ALT1,  ADC_3, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 3, 0)}, // ADC3_IN3 // Connected to ARD_A3 [ADC]
    {PC_3,       ADC_1, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 4, 0)}, // ADC1_IN4 // Connected to ARD_A2 [ADC]
    {PC_3_ALT0,  ADC_2, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 4, 0)}, // ADC2_IN4 // Connected to ARD_A2 [ADC]
    {PC_3_ALT1,  ADC_3, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 4, 0)}, // ADC3_IN4 // Connected to ARD_A2 [ADC]
    {PC_4,       ADC_1, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 13, 0)}, // ADC1_IN13 // Connected to ARD_A1 [ADC]
    {PC_4_ALT0,  ADC_2, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 13, 0)}, // ADC2_IN13 // Connected to ARD_A1 [ADC]
    {PC_5,       ADC_1, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 14, 0)}, // ADC1_IN14 // Connected to ARD_A0 [ADC]
    {PC_5_ALT0,  ADC_2, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 14, 0)}, // ADC2_IN14 // Connected to ARD_A0 [ADC]
    {NC, NC, 0}
};

MSTD_CONSTEXPR_OBJ_11 PinMap PinMap_ADC_Internal[] = {
    {ADC_TEMP,   ADC_1,    STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 17, 0)},
    {ADC_VREF,   ADC_1,    STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0,  0, 0)},
    {ADC_VBAT,   ADC_1,    STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 18, 0)},
    {NC, NC, 0}
};

//*** DAC ***

MSTD_CONSTEXPR_OBJ_11 PinMap PinMap_DAC[] = {
    {PA_4,       DAC_1, STM_PIN_DATA_EXT(STM_MODE_ANALOG, GPIO_NOPULL, 0, 1, 0)}, // DAC1_OUT1 // Connected to ARD_D7
    {PA_5,       DAC_1, STM_PIN_DATA_EXT(STM_MODE_ANALOG, GPIO_NOPULL, 0, 2, 0)}, // DAC1_OUT2 // Connected to ARD_D13 [SPI1_SCK]
    {NC, NC, 0}
};

//*** I2C ***

MSTD_CONSTEXPR_OBJ_11 PinMap PinMap_I2C_SDA[] = {
//  {PB_7,       I2C_1, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF4_I2C1)}, // Connected to STDIO_UART_RX
    {PB_9,       I2C_1, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF4_I2C1)}, // Connected to ARD_D14 [I2C1_SDA]
    {PB_11,      I2C_2, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF4_I2C2)}, // Connected to INTERNAL_I2C2_SDA [VL53L0X_SDA]
    {PB_14,      I2C_2, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF4_I2C2)}, // Connected to LED2 [LED_GREEN]
    {PC_1,       I2C_3, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF4_I2C3)}, // Connected to ARD_A4 [ADC]
    {NC, NC, 0}
};

MSTD_CONSTEXPR_OBJ_11 PinMap PinMap_I2C_SCL[] = {
//  {PB_6,       I2C_1, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF4_I2C1)}, // Connected to STDIO_UART_TX
    {PB_8,       I2C_1, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF4_I2C1)}, // Connected to ARD_D15 [I2C1_SCL]
    {PB_10,      I2C_2, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF4_I2C2)}, // Connected to INTERNAL_I2C2_SCL [VL53L0X_SCL]
    {PB_13,      I2C_2, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF4_I2C2)}, // Connected to ISM43362_WAKEUP [ISM43362_WKUP]
    {PC_0,       I2C_3, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF4_I2C3)}, // Connected to ARD_A5 [ADC]
    {NC, NC, 0}
};

//*** PWM ***

// TIM5 cannot be used because already used by the us_ticker
MSTD_CONSTEXPR_OBJ_11 PinMap PinMap_PWM[] = {
    {PA_0,       PWM_2,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM2, 1, 0)}, // TIM2_CH1 // Connected to ARD_D1 [UART4_TX]
//  {PA_0,       PWM_5,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM5, 1, 0)}, // TIM5_CH1 // Connected to ARD_D1 [UART4_TX]
    {PA_1,       PWM_2,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM2, 2, 0)}, // TIM2_CH2 // Connected to ARD_D0 [UART4_RX]
//  {PA_1,       PWM_5,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM5, 2, 0)}, // TIM5_CH2 // Connected to ARD_D0 [UART4_RX]
    {PA_1_ALT0,  PWM_15, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF14_TIM15, 1, 1)}, // TIM15_CH1N // Connected to ARD_D0 [UART4_RX]
    {PA_2,       PWM_2,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM2, 3, 0)}, // TIM2_CH3 // Connected to ARD_D10 [SPI_SSN]
//  {PA_2,       PWM_5,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM5, 3, 0)}, // TIM5_CH3 // Connected to ARD_D10 [SPI_SSN]
    {PA_2_ALT0,  PWM_15, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF14_TIM15, 1, 0)}, // TIM15_CH1 // Connected to ARD_D10 [SPI_SSN]
    {PA_3,       PWM_2,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM2, 4, 0)}, // TIM2_CH4 // Connected to ARD_D4
//  {PA_3,       PWM_5,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM5, 4, 0)}, // TIM5_CH4 // Connected to ARD_D4
    {PA_3_ALT0,  PWM_15, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF14_TIM15, 2, 0)}, // TIM15_CH2 // Connected to ARD_D4
    {PA_5,       PWM_2,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM2, 1, 0)}, // TIM2_CH1 // Connected to ARD_D13 [SPI1_SCK]
    {PA_5_ALT0,  PWM_8,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF3_TIM8, 1, 1)}, // TIM8_CH1N // Connected to ARD_D13 [SPI1_SCK]
    {PA_6,       PWM_3,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM3, 1, 0)}, // TIM3_CH1 // Connected to ARD_D12 [SPI1_MISO]
    {PA_6_ALT0,  PWM_16, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF14_TIM16, 1, 0)}, // TIM16_CH1 // Connected to ARD_D12 [SPI1_MISO]
    {PA_7,       PWM_1,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM1, 1, 1)}, // TIM1_CH1N // Connected to ARD_D11 [SPI1_MOSI]
    {PA_7_ALT0,  PWM_3,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM3, 2, 0)}, // TIM3_CH2 // Connected to ARD_D11 [SPI1_MOSI]
    {PA_7_ALT1,  PWM_8,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF3_TIM8, 1, 1)}, // TIM8_CH1N // Connected to ARD_D11 [SPI1_MOSI]
    {PA_7_ALT2,  PWM_17, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF14_TIM17, 1, 0)}, // TIM17_CH1 // Connected to ARD_D11 [SPI1_MOSI]
    {PA_8,       PWM_1,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM1, 1, 0)}, // TIM1_CH1 // Connected to SPBTLE_RF_RST
    {PA_9,       PWM_1,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM1, 2, 0)}, // TIM1_CH2 // Connected to USB_OTG_FS_VBUS [STMPS2141STR_OUT]
    {PA_10,      PWM_1,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM1, 3, 0)}, // TIM1_CH3 // Connected to USB_OTG_FS_ID
    {PA_11,      PWM_1,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM1, 4, 0)}, // TIM1_CH4 // Connected to USB_OTG_FS_DM
    {PA_15,      PWM_2,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM2, 1, 0)}, // TIM2_CH1
    {PB_0,       PWM_1,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM1, 2, 1)}, // TIM1_CH2N // Connected to ARD_D3 [INT_EXT10]
    {PB_0_ALT0,  PWM_3,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM3, 3, 0)}, // TIM3_CH3 // Connected to ARD_D3 [INT_EXT10]
    {PB_0_ALT1,  PWM_8,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF3_TIM8, 2, 1)}, // TIM8_CH2N // Connected to ARD_D3 [INT_EXT10]
    {PB_1,       PWM_1,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM1, 3, 1)}, // TIM1_CH3N // Connected to ARD_D6 [ADC1_IN6]
    {PB_1_ALT0,  PWM_3,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM3, 4, 0)}, // TIM3_CH4 // Connected to ARD_D6 [ADC1_IN6]
    {PB_1_ALT1,  PWM_8,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF3_TIM8, 3, 1)}, // TIM8_CH3N // Connected to ARD_D6 [ADC1_IN6]
    {PB_3,       PWM_2,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM2, 2, 0)}, // TIM2_CH2
    {PB_4,       PWM_3,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM3, 1, 0)}, // TIM3_CH1
    {PB_5,       PWM_3,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM3, 2, 0)}, // TIM3_CH2 // Connected to SPSGRF_915_SPI3_CSN [SPSGRF_SPI_CS]
//  {PB_6,       PWM_4,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM4, 1, 0)}, // TIM4_CH1 // Connected to STDIO_UART_TX
//  {PB_6,       PWM_16, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF14_TIM16, 1, 1)}, // TIM16_CH1N // Connected to STDIO_UART_TX
//  {PB_7,       PWM_4,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM4, 2, 0)}, // TIM4_CH2 // Connected to STDIO_UART_RX
//  {PB_7,       PWM_17, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF14_TIM17, 1, 1)}, // TIM17_CH1N // Connected to STDIO_UART_RX
    {PB_8,       PWM_4,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM4, 3, 0)}, // TIM4_CH3 // Connected to ARD_D15 [I2C1_SCL]
    {PB_8_ALT0,  PWM_16, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF14_TIM16, 1, 0)}, // TIM16_CH1 // Connected to ARD_D15 [I2C1_SCL]
    {PB_9,       PWM_4,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM4, 4, 0)}, // TIM4_CH4 // Connected to ARD_D14 [I2C1_SDA]
    {PB_9_ALT0,  PWM_17, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF14_TIM17, 1, 0)}, // TIM17_CH1 // Connected to ARD_D14 [I2C1_SDA]
    {PB_10,      PWM_2,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM2, 3, 0)}, // TIM2_CH3 // Connected to INTERNAL_I2C2_SCL [VL53L0X_SCL]
    {PB_11,      PWM_2,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM2, 4, 0)}, // TIM2_CH4 // Connected to INTERNAL_I2C2_SDA [VL53L0X_SDA]
    {PB_13,      PWM_1,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM1, 1, 1)}, // TIM1_CH1N // Connected to ISM43362_WAKEUP [ISM43362_WKUP]
    {PB_13_ALT0, PWM_15, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF14_TIM15, 1, 1)}, // TIM15_CH1N // Connected to ISM43362_WAKEUP [ISM43362_WKUP]
    {PB_14,      PWM_1,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM1, 2, 1)}, // TIM1_CH2N // Connected to LED2 [LED_GREEN]
    {PB_14_ALT0, PWM_8,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF3_TIM8, 2, 1)}, // TIM8_CH2N // Connected to LED2 [LED_GREEN]
    {PB_14_ALT1, PWM_15, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF14_TIM15, 1, 0)}, // TIM15_CH1 // Connected to LED2 [LED_GREEN]
    {PB_15,      PWM_1,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM1, 3, 1)}, // TIM1_CH3N // Connected to SPSGRF_915_SDN [SPSGRF_SDN]
    {PB_15_ALT0, PWM_8,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF3_TIM8, 3, 1)}, // TIM8_CH3N // Connected to SPSGRF_915_SDN [SPSGRF_SDN]
    {PB_15_ALT1, PWM_15, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF14_TIM15, 2, 0)}, // TIM15_CH2 // Connected to SPSGRF_915_SDN [SPSGRF_SDN]
    {PC_6,       PWM_3,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM3, 1, 0)}, // TIM3_CH1 // Connected to VL53L0X_XSHUT [Read_XSHUT]
    {PC_6_ALT0,  PWM_8,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF3_TIM8, 1, 0)}, // TIM8_CH1 // Connected to VL53L0X_XSHUT [Read_XSHUT]
    {PC_7,       PWM_3,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM3, 2, 0)}, // TIM3_CH2 // Connected to VL53L0X_GPIO1_EXTI7 [VL53L0X_GPIO1]
    {PC_7_ALT0,  PWM_8,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF3_TIM8, 2, 0)}, // TIM8_CH2 // Connected to VL53L0X_GPIO1_EXTI7 [VL53L0X_GPIO1]
    {PC_8,       PWM_3,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM3, 3, 0)}, // TIM3_CH3 // Connected to LSM3MDL_DRDY_EXTI8 [LIS3MDL_DRDY]
    {PC_8_ALT0,  PWM_8,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF3_TIM8, 3, 0)}, // TIM8_CH3 // Connected to LSM3MDL_DRDY_EXTI8 [LIS3MDL_DRDY]
    {PC_9,       PWM_3,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM3, 4, 0)}, // TIM3_CH4 // Connected to LED3_WIFI_ LED4_BLE
    {PC_9_ALT0,  PWM_8,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF3_TIM8, 4, 0)}, // TIM8_CH4 // Connected to LED3_WIFI_ LED4_BLE
    {PD_12,      PWM_4,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM4, 1, 0)}, // TIM4_CH1 // Connected to USB_OTG_FS_PWR_EN [STMPS2141STR_EN]
    {PD_13,      PWM_4,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM4, 2, 0)}, // TIM4_CH2 // Connected to SPBTLE_RF_SPI3_CSN [BT module_SPI_CS]
    {PD_14,      PWM_4,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM4, 3, 0)}, // TIM4_CH3 // Connected to ARD_D2 [INT0_EXTI14]
    {PD_15,      PWM_4,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM4, 4, 0)}, // TIM4_CH4 // Connected to HTS221_DRDY_EXTI15 [HTS221_DRDY]
    {PE_0,       PWM_16, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF14_TIM16, 1, 0)}, // TIM16_CH1 // Connected to ISM43362_SPI3_CSN [ISM43362_SSN]
    {PE_1,       PWM_17, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF14_TIM17, 1, 0)}, // TIM17_CH1 // Connected to ISM43362_DRDY_EXTI1 [ISM43362_DATARDY]
    {PE_3,       PWM_3,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM3, 1, 0)}, // TIM3_CH1 // Connected to USB_OTG_FS_OVRCR_EXTI3 [STMPS2141STR_FAULT]
    {PE_4,       PWM_3,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM3, 2, 0)}, // TIM3_CH2 // Connected to M24SR64_Y_GPO [M24SR64_GPO]
    {PE_5,       PWM_3,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM3, 3, 0)}, // TIM3_CH3 // Connected to SPSGRF_915_GPIO3_EXTI5 [SPSGRF_GPIO_3]
    {PE_6,       PWM_3,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM3, 4, 0)}, // TIM3_CH4 // Connected to SPBTLE_RF_IRQ_EXTI6 [BT module_SPI_IRQ]
    {PE_8,       PWM_1,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM1, 1, 1)}, // TIM1_CH1N // Connected to ISM43362_RST [ISM43362_RSTN]
    {PE_9,       PWM_1,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM1, 1, 0)}, // TIM1_CH1 // Connected to DFSDM1_CKOUT [MP34DT01_CLK]
    {PE_10,      PWM_1,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM1, 2, 1)}, // TIM1_CH2N // Connected to QUADSPI_CLK [MX25R6435F_SCLK]
    {PE_11,      PWM_1,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM1, 2, 0)}, // TIM1_CH2 // Connected to QUADSPI_NCS [MX25R6435F_SCLK]
    {PE_12,      PWM_1,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM1, 3, 1)}, // TIM1_CH3N // Connected to OQUADSPI_BK1_IO0 [MX25R6435F_IO0]
    {PE_13,      PWM_1,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM1, 3, 0)}, // TIM1_CH3 // Connected to QUADSPI_BK1_IO1 [MX25R6435F_IO1]
    {PE_14,      PWM_1,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM1, 4, 0)}, // TIM1_CH4 // Connected to QUAD_SPI_BK1_IO2 [MX25R6435F_IO2]
    {NC, NC, 0}
};

//*** SERIAL ***

MSTD_CONSTEXPR_OBJ_11 PinMap PinMap_UART_TX[] = {
    {PA_0,       UART_4,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF8_UART4)}, // Connected to ARD_D1 [UART4_TX]
    {PA_2,       UART_2,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART2)}, // Connected to ARD_D10 [SPI_SSN]
    {PA_9,       UART_1,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART1)}, // Connected to USB_OTG_FS_VBUS [STMPS2141STR_OUT]
    {PB_6,       UART_1,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART1)}, // Connected to STDIO_UART_TX
    {PB_10,      UART_3,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART3)}, // Connected to INTERNAL_I2C2_SCL [VL53L0X_SCL]
    {PB_11,      LPUART_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF8_LPUART1)}, // Connected to INTERNAL_I2C2_SDA [VL53L0X_SDA]
    {PC_1,       LPUART_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF8_LPUART1)}, // Connected to ARD_A4 [ADC]
    {PC_4,       UART_3,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART3)}, // Connected to ARD_A1 [ADC]
    {PC_10,      UART_3,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART3)}, // Connected to INTERNAL_SPI3_SCK [BT module_SPI_SCLK]
    {PC_10_ALT0, UART_4,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF8_UART4)}, // Connected to INTERNAL_SPI3_SCK [BT module_SPI_SCLK]
    {PC_12,      UART_5,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF8_UART5)}, // Connected to INTERNAL_SPI3_MOSI [BT module_SPI_MOSI]
    {PD_5,       UART_2,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART2)}, // Connected to PMOD_UART2_TX
    {PD_8,       UART_3,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART3)}, // Connected to INTERNAL_UART3_TX [ISM43362_RX]
    {NC, NC, 0}
};

MSTD_CONSTEXPR_OBJ_11 PinMap PinMap_UART_RX[] = {
    {PA_1,       UART_4,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF8_UART4)}, // Connected to ARD_D0 [UART4_RX]
    {PA_3,       UART_2,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART2)}, // Connected to ARD_D4
    {PA_10,      UART_1,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART1)}, // Connected to USB_OTG_FS_ID
    {PB_7,       UART_1,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART1)}, // Connected to STDIO_UART_RX
    {PB_10,      LPUART_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF8_LPUART1)}, // Connected to INTERNAL_I2C2_SCL [VL53L0X_SCL]
    {PB_11,      UART_3,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART3)}, // Connected to INTERNAL_I2C2_SDA [VL53L0X_SDA]
    {PC_0,       LPUART_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF8_LPUART1)}, // Connected to ARD_A5 [ADC]
    {PC_5,       UART_3,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART3)}, // Connected to ARD_A0 [ADC]
    {PC_11,      UART_3,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART3)}, // Connected to INTERNAL_SPI3_MISO [BT module_SPI_MISO]
    {PC_11_ALT0, UART_4,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF8_UART4)}, // Connected to INTERNAL_SPI3_MISO [BT module_SPI_MISO]
    {PD_2,       UART_5,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF8_UART5)}, // Connected to PMOD_IRQ_EXTI12
    {PD_6,       UART_2,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART2)}, // Connected to PMOD_UART2_RX
    {PD_9,       UART_3,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART3)}, // Connected to INTERNAL_UART3_RX [ISM43362_TX]
    {NC, NC, 0}
};

MSTD_CONSTEXPR_OBJ_11 PinMap PinMap_UART_RTS[] = {
    {PA_1,       UART_2,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART2)}, // Connected to ARD_D0 [UART4_RX]
    {PA_12,      UART_1,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART1)}, // Connected to USB_OTG_FS_DP
    {PA_15,      UART_4,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF8_UART4)},
    {PB_1,       UART_3,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART3)}, // Connected to ARD_D6 [ADC1_IN6]
    {PB_3,       UART_1,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART1)},
    {PB_4,       UART_5,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF8_UART5)},
    {PB_12,      LPUART_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF8_LPUART1)}, // Connected to ISM43362_BOOT0 [ISM43362_BOOT]
    {PB_14,      UART_3,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART3)}, // Connected to LED2 [LED_GREEN]
    {PD_2,       UART_3,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART3)}, // Connected to PMOD_IRQ_EXTI12
    {PD_4,       UART_2,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART2)}, // Connected to PMOD_UART2_RTS
    {PD_12,      UART_3,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART3)}, // Connected to USB_OTG_FS_PWR_EN [STMPS2141STR_EN]
    {NC, NC, 0}
};

MSTD_CONSTEXPR_OBJ_11 PinMap PinMap_UART_CTS[] = {
    {PA_0,       UART_2,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART2)}, // Connected to ARD_D1 [UART4_TX]
    {PA_6,       UART_3,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART3)}, // Connected to ARD_D12 [SPI1_MISO]
    {PA_11,      UART_1,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART1)}, // Connected to USB_OTG_FS_DM
    {PB_4,       UART_1,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART1)},
    {PB_5,       UART_5,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF8_UART5)}, // Connected to SPSGRF_915_SPI3_CSN [SPSGRF_SPI_CS]
    {PB_7,       UART_4,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF8_UART4)}, // Connected to STDIO_UART_RX
    {PB_13,      UART_3,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART3)}, // Connected to ISM43362_WAKEUP [ISM43362_WKUP]
    {PB_13_ALT0, LPUART_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF8_LPUART1)}, // Connected to ISM43362_WAKEUP [ISM43362_WKUP]
    {PD_3,       UART_2,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART2)}, // Connected to PMOD_UART2_CTS
    {PD_11,      UART_3,  STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART3)}, // Connected to LSM6DSL_INT1_EXTI11 [LSM6DSL_INT1]
    {NC, NC, 0}
};

//*** SPI ***

MSTD_CONSTEXPR_OBJ_11 PinMap PinMap_SPI_MOSI[] = {
    {PA_7,       SPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI1)}, // Connected to ARD_D11 [SPI1_MOSI]
    {PB_5,       SPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI1)}, // Connected to SPSGRF_915_SPI3_CSN [SPSGRF_SPI_CS]
    {PB_5_ALT0,  SPI_3, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF6_SPI3)}, // Connected to SPSGRF_915_SPI3_CSN [SPSGRF_SPI_CS]
    {PB_15,      SPI_2, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI2)}, // Connected to SPSGRF_915_SDN [SPSGRF_SDN]
    {PC_3,       SPI_2, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI2)}, // Connected to ARD_A2 [ADC]
    {PC_12,      SPI_3, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF6_SPI3)}, // Connected to INTERNAL_SPI3_MOSI [BT module_SPI_MOSI]
    {PD_4,       SPI_2, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI2)}, // Connected to PMOD_UART2_RTS
    {PE_15,      SPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI1)}, // Connected to QUAD_SPI_BK1_IO3 [MX25R6435F_IO3]
    {NC, NC, 0}
};

MSTD_CONSTEXPR_OBJ_11 PinMap PinMap_SPI_MISO[] = {
    {PA_6,       SPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI1)}, // Connected to ARD_D12 [SPI1_MISO]
    {PB_4,       SPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI1)},
    {PB_4_ALT0,  SPI_3, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF6_SPI3)},
    {PB_14,      SPI_2, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI2)}, // Connected to LED2 [LED_GREEN]
    {PC_2,       SPI_2, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI2)}, // Connected to ARD_A3 [ADC]
    {PC_11,      SPI_3, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF6_SPI3)}, // Connected to INTERNAL_SPI3_MISO [BT module_SPI_MISO]
    {PD_3,       SPI_2, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI2)}, // Connected to PMOD_UART2_CTS
    {PE_14,      SPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI1)}, // Connected to QUAD_SPI_BK1_IO2 [MX25R6435F_IO2]
    {NC, NC, 0}
};

MSTD_CONSTEXPR_OBJ_11 PinMap PinMap_SPI_SCLK[] = {
    {PA_5,       SPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI1)}, // Connected to ARD_D13 [SPI1_SCK]
    {PB_3,       SPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI1)},
    {PB_3_ALT0,  SPI_3, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF6_SPI3)},
    {PB_10,      SPI_2, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI2)}, // Connected to INTERNAL_I2C2_SCL [VL53L0X_SCL]
    {PB_13,      SPI_2, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI2)}, // Connected to ISM43362_WAKEUP [ISM43362_WKUP]
    {PC_10,      SPI_3, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF6_SPI3)}, // Connected to INTERNAL_SPI3_SCK [BT module_SPI_SCLK]
    {PD_1,       SPI_2, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI2)}, // Connected to PMOD_SPI2_SCK
    {PE_13,      SPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI1)}, // Connected to QUADSPI_BK1_IO1 [MX25R6435F_IO1]
    {NC, NC, 0}
};

MSTD_CONSTEXPR_OBJ_11 PinMap PinMap_SPI_SSEL[] = {
    {PA_4,       SPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI1)}, // Connected to ARD_D7
    {PA_4_ALT0,  SPI_3, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF6_SPI3)}, // Connected to ARD_D7
    {PA_15,      SPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI1)},
    {PA_15_ALT0, SPI_3, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF6_SPI3)},
    {PB_9,       SPI_2, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI2)}, // Connected to ARD_D14 [I2C1_SDA]
    {PB_12,      SPI_2, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI2)}, // Connected to ISM43362_BOOT0 [ISM43362_BOOT]
    {PD_0,       SPI_2, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI2)}, // Connected to PMOD_RESET
    {PE_12,      SPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI1)}, // Connected to OQUADSPI_BK1_IO0 [MX25R6435F_IO0]
    {NC, NC, 0}
};

//*** CAN ***

MSTD_CONSTEXPR_OBJ_11 PinMap PinMap_CAN_RD[] = {
    {PA_11,      CAN_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF9_CAN1)}, // Connected to USB_OTG_FS_DM
    {PB_8,       CAN_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF9_CAN1)}, // Connected to ARD_D15 [I2C1_SCL]
    {PD_0,       CAN_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF9_CAN1)}, // Connected to PMOD_RESET
    {NC, NC, 0}
};

MSTD_CONSTEXPR_OBJ_11 PinMap PinMap_CAN_TD[] = {
    {PA_12,      CAN_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF9_CAN1)}, // Connected to USB_OTG_FS_DP
    {PB_9,       CAN_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF9_CAN1)}, // Connected to ARD_D14 [I2C1_SDA]
    {PD_1,       CAN_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF9_CAN1)}, // Connected to PMOD_SPI2_SCK
    {NC, NC, 0}
};

//*** QUADSPI ***

MSTD_CONSTEXPR_OBJ_11 PinMap PinMap_QSPI_DATA0[] = {
    {PB_1,      QSPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF10_QUADSPI)},  // QUADSPI_BK1_IO0 // Connected to ARD_D6 [ADC1_IN6]
    {PE_12,     QSPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF10_QUADSPI)},  // QUADSPI_BK1_IO0 // Connected to OQUADSPI_BK1_IO0 [MX25R6435F_IO0]
    {NC, NC, 0}
};

MSTD_CONSTEXPR_OBJ_11 PinMap PinMap_QSPI_DATA1[] = {
    {PB_0,      QSPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF10_QUADSPI)},  // QUADSPI_BK1_IO1 // Connected to ARD_D3 [INT_EXT10]
    {PE_13,     QSPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF10_QUADSPI)},  // QUADSPI_BK1_IO1 // Connected to QUADSPI_BK1_IO1 [MX25R6435F_IO1]
    {NC, NC, 0}
};

MSTD_CONSTEXPR_OBJ_11 PinMap PinMap_QSPI_DATA2[] = {
    {PA_7,      QSPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF10_QUADSPI)},  // QUADSPI_BK1_IO2 // Connected to ARD_D11 [SPI1_MOSI]
    {PE_14,     QSPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF10_QUADSPI)},  // QUADSPI_BK1_IO2 // Connected to QUAD_SPI_BK1_IO2 [MX25R6435F_IO2]
    {NC, NC, 0}
};

MSTD_CONSTEXPR_OBJ_11 PinMap PinMap_QSPI_DATA3[] = {
    {PA_6,      QSPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF10_QUADSPI)},  // QUADSPI_BK1_IO3 // Connected to ARD_D12 [SPI1_MISO]
    {PE_15,     QSPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF10_QUADSPI)},  // QUADSPI_BK1_IO3 // Connected to QUAD_SPI_BK1_IO3 [MX25R6435F_IO3]
    {NC, NC, 0}
};

MSTD_CONSTEXPR_OBJ_11 PinMap PinMap_QSPI_SCLK[] = {
    {PB_10,     QSPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF10_QUADSPI)},  // QUADSPI_CLK // Connected to INTERNAL_I2C2_SCL [VL53L0X_SCL]
    {PE_10,     QSPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF10_QUADSPI)},  // QUADSPI_CLK // Connected to QUADSPI_CLK [MX25R6435F_SCLK]
    {NC, NC, 0}
};

MSTD_CONSTEXPR_OBJ_11 PinMap PinMap_QSPI_SSEL[] = {
    {PB_11,     QSPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF10_QUADSPI)},  // QUADSPI_NCS // Connected to INTERNAL_I2C2_SDA [VL53L0X_SDA]
    {PE_11,     QSPI_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF10_QUADSPI)},  // QUADSPI_NCS // Connected to QUADSPI_NCS [MX25R6435F_SCLK]
    {NC, NC, 0}
};

//*** USBDEVICE ***

MSTD_CONSTEXPR_OBJ_11 const PinMap PinMap_USB_FS[] = {
//  {PA_8,      USB_FS, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF10_OTG_FS)}, // USB_OTG_FS_SOF // Connected to SPBTLE_RF_RST
    {PA_9,      USB_FS, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, GPIO_AF_NONE)}, // USB_OTG_FS_VBUS // Connected to USB_OTG_FS_VBUS [STMPS2141STR_OUT]
    {PA_10,     USB_FS, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_PULLUP, GPIO_AF10_OTG_FS)}, // USB_OTG_FS_ID // Connected to USB_OTG_FS_ID
    {PA_11,     USB_FS, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF10_OTG_FS)}, // USB_OTG_FS_DM // Connected to USB_OTG_FS_DM
    {PA_12,     USB_FS, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF10_OTG_FS)}, // USB_OTG_FS_DP // Connected to USB_OTG_FS_DP
//  {PA_13,     USB_FS, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF10_OTG_FS)}, // USB_OTG_FS_NOE
//  {PC_9,      USB_FS, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF10_OTG_FS)}, // USB_OTG_FS_NOE // Connected to LED3_WIFI_ LED4_BLE
    {NC, NC, 0}
};

#define PINMAP_ANALOGIN PinMap_ADC
#define PINMAP_ANALOGIN_INTERNAL PinMap_ADC_Internal
#define PINMAP_ANALOGOUT PinMap_DAC
#define PINMAP_I2C_SDA PinMap_I2C_SDA
#define PINMAP_I2C_SCL PinMap_I2C_SCL
#define PINMAP_UART_TX PinMap_UART_TX
#define PINMAP_UART_RX PinMap_UART_RX
#define PINMAP_UART_CTS PinMap_UART_CTS
#define PINMAP_UART_RTS PinMap_UART_RTS
#define PINMAP_SPI_SCLK PinMap_SPI_SCLK
#define PINMAP_SPI_MOSI PinMap_SPI_MOSI
#define PINMAP_SPI_MISO PinMap_SPI_MISO
#define PINMAP_SPI_SSEL PinMap_SPI_SSEL
#define PINMAP_PWM PinMap_PWM
#define PINMAP_QSPI_DATA0 PinMap_QSPI_DATA0
#define PINMAP_QSPI_DATA1 PinMap_QSPI_DATA1
#define PINMAP_QSPI_DATA2 PinMap_QSPI_DATA2
#define PINMAP_QSPI_DATA3 PinMap_QSPI_DATA3
#define PINMAP_QSPI_SCLK PinMap_QSPI_SCLK
#define PINMAP_QSPI_SSEL PinMap_QSPI_SSEL
#define PINMAP_CAN_RD PinMap_CAN_RD
#define PINMAP_CAN_TD PinMap_CAN_RD

#endif