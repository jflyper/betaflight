/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include "platform.h"
#include "drivers/io.h"

#include "drivers/timer.h"
#include "drivers/timer_def.h"
#include "drivers/dma.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
//    DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_MOTOR,             0, 0, 4),

    DEF_TIM(TIM3,  CH1, PC6,  TIM_USE_MOTOR,             0, 0, 4), // T3C1, T8C1
    DEF_TIM(TIM3,  CH2, PC7,  TIM_USE_MOTOR,             0, 1, 4), // T3C2, T8C2
    DEF_TIM(TIM3,  CH3, PC8,  TIM_USE_MOTOR,             0, 2, 4), // T3C3, T8C3, T20C3
    DEF_TIM(TIM3,  CH4, PC9,  TIM_USE_MOTOR,             0, 3, 4), // T3C4, T8C4

    DEF_TIM(TIM2,  CH1, PA0,  TIM_USE_NONE,              0, 4, 4), // can be TIM5_CH1
    DEF_TIM(TIM5,  CH2, PA1,  TIM_USE_NONE,              0, 5, 4), // can be TIM5_CH2

    DEF_TIM(TIM1,  CH1, PC0,  TIM_USE_NONE,              0, 5, 4), // PWM ok
    DEF_TIM(TIM1,  CH2, PC1,  TIM_USE_NONE,              0, 5, 4), // PWM ok
    DEF_TIM(TIM1,  CH3, PC2,  TIM_USE_NONE,              0, 5, 4), // PWM ok
    DEF_TIM(TIM1,  CH4, PC3,  TIM_USE_NONE,              0, 5, 4), // PWM ok

    // DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_NONE,              0, 5, 4), // PWM ok
    // DEF_TIM(TIM3,  CH2, PA4,  TIM_USE_NONE,              0, 5, 4), // PWM ok
    // DEF_TIM(TIM20, CH1, PB2,  TIM_USE_NONE,              0, 5, 4), // PWM ok T20C1

// TIM4
    DEF_TIM(TIM4,  CH2, PB7,  TIM_USE_LED,               0, 8, 4),

// TIM5

// TIM8
    // DEF_TIM(TIM8,  CH3, PB9,  TIM_USE_NONE,              0, 0, 0), // T4C4, T8C3, T17C1

// TIM15
    // DEF_TIM(TIM15,  CH1, PA2,  TIM_USE_NONE,              0, 0, 0), // T2C3, T5C3, T15C1
    // DEF_TIM(TIM15,  CH1, PA3,  TIM_USE_NONE,              0, 0, 0), // T2C4, T5C4, T15C2

// TIM16

// TIM17
    // DEF_TIM(TIM17,  CH1, PB5,  TIM_USE_NONE,              0, 0, 0), // T3C2, T17C1
};
