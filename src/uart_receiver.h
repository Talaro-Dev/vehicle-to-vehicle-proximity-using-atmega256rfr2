#ifndef UART_RECEIVER_H
#define UART_RECEIVER_H

#include <avr/io.h>
#include <stdint.h>

static void uart_init(void)
{
    UCSR1A = 0; //normal speed mode
    UBRR1 = 12; // baud rate ~38400
    UCSR1B = _BV(TXEN1); //turn on the transmitter only
    UCSR1C = _BV(UCSZ11) | _BV(UCSZ10); //8-bit data, no parity, 1 stop bit
}

static void uart_putc(char c)
{
    while(!(UCSR1A & _BV(UDRE1))) {
        // wait until the transmit buffer is empty
    }
    UDR1 = c; // send the character
}

static void uart_write(const char *text)
{
    while(*text != '\0'){
        uart_putc(*text);
        text++;
    }
}

static void uart_print_uint(uint32_t value)
{
    char buffer[11];
    uint8_t i = 0;
    if(value == 0){
        uart_putc('0');
        return;
    }
    while(value > 0){
        buffer[i] = '0' + (value % 10);
        value /= 10;
        i++;
    }
    while(i > 0){
        i--;
        uart_putc(buffer[i]);
    }
    return;
}

static void uart_print_int(int16_t value)
{
    if(value < 0){
        uart_putc('-');
        value = -value;
    }
    uart_print_uint((uint16_t)value);
    return;
}

static uint16_t read_u16(volatile uint8_t *buffer, uint8_t position)
{
    return buffer[position] | ((uint16_t)buffer[position + 1] << 8);
}

#endif