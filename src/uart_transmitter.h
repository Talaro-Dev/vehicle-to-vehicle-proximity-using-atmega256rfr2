#ifndef UART_TRANSMITTER_H
#define UART_TRANSMITTER_H

#include <avr/io.h>
#include <stdint.h>
#include "variables_transmitter.h"

static void uart_init(void)
{
    UCSR1A = 0;
    UBRR1 = 12;
    UCSR1B = _BV(TXEN1);
    UCSR1C = _BV(UCSZ11) | _BV(UCSZ10);
}

static void uart_putc(char c)
{
    while(!(UCSR1A & _BV(UDRE1))){}
    UDR1 = c;
}

static void uart_write(const char *text)
{
    while(*text != '\0')
    {
        uart_putc(*text);
        text++;
    }
}

static void uart_print_uint(uint16_t value)
{
    char digits[5];
    uint8_t count = 0;

    do
    {
        digits[count] = '0' + (value % 10);
        count++;
        value /= 10;
    } while(value != 0);

    while (count > 0)
    {
        count--;
        uart_putc(digits[count]);
    }
}

#endif