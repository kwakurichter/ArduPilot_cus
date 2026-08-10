/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  Crazyflie 2.1 brushless deck quirks for the ChibiOS RCOutput driver.

  Everything board-specific about driving that deck's ESCs lives here rather
  than scattered through RCOutput.cpp and RCOutput_bdshot.cpp, so those files
  carry only unconditional one-line calls and stay easy to rebase onto new
  upstream releases. When HAL_CF21_BRUSHLESS is undefined the hooks are empty
  inline functions in RCOutput.h and this file compiles to nothing.

  Two quirks are handled:

    - The four motor lines hang off TIM2 and must stay open-drain with a
      pullup. The generic bidirectional DShot path reconfigures them push-pull
      at MID2 speed, so the mode has to be re-asserted around every transmit
      and receive transition.

    - The ESCs share a reset line on PC15 which must be pulsed when entering
      bidirectional DShot.

  A third quirk, that TIM2_CH4 can never do input capture, is not handled here.
  It is a property of the DMA stream map rather than of this deck, so it is
  expressed by HAL_BDSHOT_NO_SHARE_UP_STREAM in the hwdef instead. See
  bdshot_setup_group_ic_DMA().
 */

#include <hal.h>

#if defined(HAL_CF21_BRUSHLESS) && HAL_USE_PWM == TRUE

#include "RCOutput.h"

using namespace ChibiOS;

extern const AP_HAL::HAL& hal;

bool RCOutput::cf21_is_tim2_motor_group(const pwm_group &group)
{
    return group.timer_id == 2;
}

void RCOutput::cf21_set_tim2_motor_lines_tx(const pwm_group &group, bool bidir)
{
    if (!cf21_is_tim2_motor_group(group)) {
        return;
    }

    iomode_t mode = PAL_MODE_ALTERNATE(1) | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_HIGHEST;

    // In bidirectional mode keep the line released high when the FC is not actively pulling it low
#if defined(PAL_STM32_PUPDR_PULLUP)
    if (bidir) {
        mode |= PAL_STM32_PUPDR_PULLUP;
    }
#endif

    palSetPadMode(GPIOA, 1, mode);   // M1 TIM2_CH2
    palSetPadMode(GPIOB, 11, mode);  // M2 TIM2_CH4
    palSetPadMode(GPIOA, 15, mode);  // M3 TIM2_CH1
    palSetPadMode(GPIOB, 10, mode);  // M4 TIM2_CH3
}

void RCOutput::cf21_set_tim2_motor_lines_rx(const pwm_group &group)
{
    if (!cf21_is_tim2_motor_group(group)) {
        return;
    }

    iomode_t mode = PAL_MODE_ALTERNATE(1) |  PAL_STM32_OTYPE_OPENDRAIN |  PAL_STM32_PUPDR_PULLUP |
#ifdef PAL_STM32_OSPEED_MID1
                    PAL_STM32_OSPEED_MID1;
#elif defined(PAL_STM32_OSPEED_MEDIUM)
                    PAL_STM32_OSPEED_MEDIUM;
#else
                    PAL_STM32_OSPEED_LOW;
#endif

    palSetPadMode(GPIOA, 1, mode);
    palSetPadMode(GPIOB, 11, mode);
    palSetPadMode(GPIOA, 15, mode);
    palSetPadMode(GPIOB, 10, mode);
}

void RCOutput::cf21_reset_escs_for_bdshot(const pwm_group &group)
{
    if (!cf21_is_tim2_motor_group(group)) {
        return;
    }

    // PC15 is the shared ESC reset line on the CF21 brushless deck.
    palSetPadMode(GPIOC, 15, PAL_MODE_OUTPUT_OPENDRAIN);
    palWritePad(GPIOC, 15, 0);
    hal.scheduler->delay(5);
    palWritePad(GPIOC, 15, 1);
    hal.scheduler->delay(50);
}

#endif // HAL_CF21_BRUSHLESS && HAL_USE_PWM
