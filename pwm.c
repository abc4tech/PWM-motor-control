#include "pwm.h"

/*
 * Counter 1 initialization, generate 2 PWM channels
 */
void motor_pwm_init()
{
  TCCR1A |= 0xf1;       // set PWM mode
  TCCR1A &= ~0x02;      // set PWM mode
  TCCR1B |= 0x09;       // clock source
  TCCR1B &= ~0x16;      // clock source
  OCR1AL = PWM_MAX;
  OCR1AH = 0;
  OCR1BL = PWM_MAX;
  OCR1BH = 0;
}

/*
 * Set PWM width according to ADC values on potentiometers
 */
void motor_pwm_set(unsigned char adc_data[])
{
  unsigned char motor = 0;

  while (motor < MOTOR_1 + 1) {
    if (PWM_MIN < adc_data[motor] && adc_data[motor] < PWM_MAX) {
      switch (motor) {
        case MOTOR_0:
          OCR1AL = adc_data[motor];
          break;

        case MOTOR_1:
          OCR1BL = adc_data[motor];
          break;

        default:
          break;
      }
    }
    motor++;
  }
}

void motor_stop(unsigned char motor)
{
  switch (motor) {
    case MOTOR_0:
      DDRB &= ~0x02;
      break;

    case MOTOR_1:
      DDRB &= ~0x04;
      break;
  }
}

void motor_start(unsigned char motor)
{
  switch (motor) {
    case MOTOR_0:
      DDRB |= 0x02;
      break;

    case MOTOR_1:
      DDRB |= 0x04;
      break;
  }
}