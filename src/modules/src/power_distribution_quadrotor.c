/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_quadrotor.c - Crazyflie stock power distribution code
 */

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "autoconf.h"
#include "config.h"

#ifndef CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#  define DEFAULT_IDLE_THRUST 0
#else
#  define DEFAULT_IDLE_THRUST CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

static bool motorSetEnable = false;
motors_thrust_t motorPowerSet;

void powerSet(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
  motorPowerSet.m1 = m1;
  motorPowerSet.m2 = m2;
  motorPowerSet.m3 = m3;
  motorPowerSet.m4 = m4;
}

void powerDistributionInit(void)
{
  motorPowerSet.m1 = 0;
  motorPowerSet.m2 = 0;
  motorPowerSet.m3 = 0;
  motorPowerSet.m4 = 0;
}

bool powerDistributionTest(void)
{
  bool pass = true;
  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerDistribution(motors_thrust_t* motorPower, const control_t *control)
{
  if (motorSetEnable)
  {
    motorPower->m1 = motorPowerSet.m1;
    motorPower->m2 = motorPowerSet.m2;
    motorPower->m3 = motorPowerSet.m3;
    motorPower->m4 = motorPowerSet.m4;

    return;
  }

  int16_t r = control->roll / 2.0f;
  int16_t p = control->pitch / 2.0f;
  motorPower->m1 = limitThrust(control->thrust - r + p + control->yaw);
  motorPower->m2 = limitThrust(control->thrust - r - p - control->yaw);
  motorPower->m3 =  limitThrust(control->thrust + r - p + control->yaw);
  motorPower->m4 =  limitThrust(control->thrust + r + p - control->yaw);

  if (motorPower->m1 < idleThrust) {
    motorPower->m1 = idleThrust;
  }
  if (motorPower->m2 < idleThrust) {
    motorPower->m2 = idleThrust;
  }
  if (motorPower->m3 < idleThrust) {
    motorPower->m3 = idleThrust;
  }
  if (motorPower->m4 < idleThrust) {
    motorPower->m4 = idleThrust;
  }
}

/**
 * Power distribution parameters
 */
PARAM_GROUP_START(powerDist)
/**
 * @brief Motor thrust to set at idle (default: 0)
 *
 * This is often needed for brushless motors as
 * it takes time to start up the motor. Then a
 * common value is between 3000 - 6000.
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_PERSISTENT, idleThrust, &idleThrust)
/**
 * @brief Allow override of motor power commands (default: 0)
 *
 * This enables the use of powerSet() to set motor
 * power commands directly, rather than through the
 * use of control->thrust, control->roll, etc. This
 * is done by the AE483 controller, for example.
 */
PARAM_ADD(PARAM_UINT8, motorSetEnable, &motorSetEnable)
PARAM_GROUP_STOP(powerDist)
