/*
Copyright (c) 2019 Hendrik van Wyk
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef XENON_H
#define XENON_H

#include "nrf_gpio.h"

#define LEDS_NUMBER 4

#define LED1_B NRF_GPIO_PIN_MAP(1, 12)
#define LED2_R NRF_GPIO_PIN_MAP(0, 13)
#define LED2_G NRF_GPIO_PIN_MAP(0, 14)
#define LED2_B NRF_GPIO_PIN_MAP(0, 15)

#define LED_1 LED2_R
#define LED_2 LED2_G
#define LED_3 LED2_B
#define LED_4 LED1_B

#define LEDS_ACTIVE_STATE 0

#define LEDS_LIST \
    { LED_1, LED_2, LED_3, LED_4 }

#define LEDS_INV_MASK LEDS_MASK

#define BSP_LED_0 LED_1
#define BSP_LED_1 LED_2
#define BSP_LED_2 LED_3
#define BSP_LED_3 LED_4

// Kinda hacky way of making button 1 also delete bonding info.
#define BUTTONS_NUMBER 2

#define BUTTON_1 NRF_GPIO_PIN_MAP(0, 11)
#define BUTTON_PULL NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST \
    { BUTTON_1, BUTTON_1 }

#define BSP_BUTTON_0 BUTTON_1

#endif // XENON_H
