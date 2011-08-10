/*
 * motor_pwm.c  v.1.0
 *
 * Program for PWM controlling 2 DC motors
 *
 * Michal Vokac 4.10.2010
 *
 * This program is free software: you can redistribute it and/or modify
 * it.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <avr/io.h>

#define IR_ON  PORTB |= 0x08;           /* enable LED, PB3 on */
#define IR_OFF  PORTB &= ~0x08;         /* disable LED, PB3 off */
#define PWM_MIN 20
#define PWM_MAX 220
#define FIRST_ADC 0
#define LAST_ADC 2
#define ADC_VREF_TYPE 0x60

#define MOTOR_0 0
#define MOTOR_1 1

void motor_pwm_init();
void motor_pwm_set(unsigned char *);
void motor_stop(unsigned char);
void motor_start(unsigned char);