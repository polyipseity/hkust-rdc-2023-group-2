#include "motor_c.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "can.h"

static double const input_noise_threshold = .25;

CANMotor CANMotor_new(Motor handle, bool reversed)
{
  CANMotor ret = {.m_handle = handle, .m_factor = reversed ? -1 : 1};
  return ret;
}

void CANMotor_set_input(CANMotor *self, double input)
{
  set_motor_current(self->m_handle, self->m_factor * input);
}

double CANMotor_get_input(CANMotor const *self)
{
  double const ret = self->m_factor * get_motor_feedback(self->m_handle).actual_current;
  return fabs(ret) <= input_noise_threshold ? 0. : ret;
}

double CANMotor_get_position(CANMotor const *self)
{
  double const ret = self->m_factor * get_motor_feedback(self->m_handle).encoder / 8192.;
  return ret >= 0. ? ret : 1. + ret;
}

double CANMotor_get_velocity(CANMotor const *self)
{
  return self->m_factor * get_motor_feedback(self->m_handle).vel_rpm / 60.;
}

uint8_t CANMotor_get_temperature(CANMotor const *self)
{
  return get_motor_feedback(self->m_handle).temperature;
}
