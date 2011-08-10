/*
 * motor_pwm.c	v.1.0
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
#define F_CPU 8000000UL         /*8MHz - CPU clock */
#include <avr/interrupt.h>
#include "pwm.h"

#define SENSOR_MODE 0
#define SENSOR_ON (GICR |= 0x40)        /* enable int0 */
#define SENSOR_OFF (GICR &= ~0x40)      /* disable int0 */
#define MOTOR_TIMER_STOP (TCCR0 = 0x00)
#define LED_PWM_STOP (TCCR2 = 0x00)
#define TIMER_CHANNEL 2
#define TIMER_MIN 1

volatile unsigned char adc_data[LAST_ADC - FIRST_ADC + 1];
volatile unsigned char int_wait = 0;

void motor_timer_start();
void led_pwm_init();

/*
 * SFH5110 infrared sensor external interrupt service routine
 */
ISR(INT0_vect)
{
        SENSOR_OFF;
        motor_start(MOTOR_0);
        motor_timer_start();	// start counter for enabling motor 1 output
}

/*
 * Output delay counter interrupt service routine
 */
ISR(TIMER0_OVF_vect)
{
        static unsigned int cycle = 0;

        if (cycle++ >= adc_data[TIMER_CHANNEL] + TIMER_MIN) {
                cycle = 0;
                motor_stop(MOTOR_0);
                MOTOR_TIMER_STOP;
                int_wait = 0;
        }
}

/*
 * ADC interrupt service routine
 */
ISR(ADC_vect)
{
        static unsigned char channel = 0;
        unsigned int i = 0;

        adc_data[channel] = ADCH;

        motor_pwm_set(adc_data);

        // Select next ADC input
        if (++channel > (LAST_ADC - FIRST_ADC))
                channel = 0;

        ADMUX = (FIRST_ADC | (ADC_VREF_TYPE & 0xff)) + channel;

        while(i++ < 30000);

        ADCSRA |= 0x40;
}

/*
 * Set output delay counter
 */
void motor_timer_start()
{
        TIMSK |= 0x01;        // enable overflow interrupt from counter 1
        TCCR0 = 0x05;        // prescaler 1024
        TCNT0 = 0x00;
}

/*
 * Set onfrared PWM counter
 */
void led_pwm_init()
{
        TCCR2 = 0x19;         // CTC mode, prescaler 1x
        OCR2 = 110;           // generate 36kHz
        TCNT2 = 0x00;
}

/*
 *  MAIN
 */
int main(void)
{
        DDRB = 0x0E;	// PB0 = in, PB1 - PB3 out
        PORTB |= 0x01;  // set pull-up rezistor on PB0 - output

        DDRD &= ~0x04;  // PD2 = in
        PORTD |= 0x04;  // set pull-up rezistor on PD2 - input

        motor_pwm_init();

        ADCSRA = 0xCB;
        ADMUX = 0x60;   //  Vref Vcc

        MCUCR |= 0x02;  // interupr int0 acts on falling edge

        sei();	// global interrupt enable

        motor_start(MOTOR_1);

        while(1) {
                if ((PINB & 0x01) == SENSOR_MODE && int_wait == 0) {  /* */
                        motor_stop(MOTOR_0);
                        led_pwm_init();
                        SENSOR_ON;
                        int_wait = 1;
                }
                else if ((PINB & 0x01) == !SENSOR_MODE){
                        SENSOR_OFF;
                        LED_PWM_STOP;
                        motor_start(MOTOR_0);

                        int_wait = 0;
                }
        }
        return 0;
}
