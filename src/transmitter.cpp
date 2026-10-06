#define F_CPU 8000000L
#define RF_CHANNEL 20
#define VEHICLE_PAN_ID 0x1234
#define MAC_HEADER_LENGTH 9
#define BEACON_PAYLOAD_LENGTH 6
#define FCS_LENGTH 2
#define VEHICLE_FRAME_LENGTH (MAC_HEADER_LENGTH + BEACON_PAYLOAD_LENGTH + FCS_LENGTH)

#ifndef VEHICLE_ID
#define VEHICLE_ID 2
#endif

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

static void uart_init(void)
{
    UCSR1A = 0;
    UBRR1 = 25;
    UCSR1B = _BV(TXEN1);
    UCSR1C = _BV(UCSZ11) | _BV(UCSZ10);
}

static void uart_putc(char value)
{
    while (!(UCSR1A & _BV(UDRE1))) {
    }
    UDR1 = value;
}

static void uart_write(const char *text)
{
    while (*text) {
        uart_putc(*text++);
    }
}

static void uart_print_uint(uint16_t value)
{
    char digits[5];
    uint8_t count = 0;

    do {
        digits[count++] = '0' + (value % 10);
        value /= 10;
    } while (value != 0);

    while (count != 0) {
        uart_putc(digits[--count]);
    }
}

static uint8_t wait_for_radio_state(uint8_t expected_state)
{
    uint32_t timeout = 100000UL;

    while (timeout-- != 0) {
        if ((TRX_STATUS & 0x1F) == expected_state) {
            return 1;
        }
    }
    return 0;
}

static uint8_t radio_prepare_channel(void)
{
    TRX_STATE = 0x03;
    if (!wait_for_radio_state(0x08)) {
        return 0;
    }

    PHY_CC_CCA = RF_CHANNEL & 0x1F;
    return 1;
}

static uint8_t send_vehicle_beacon(uint16_t sequence_number)
{
    if (!radio_prepare_channel()) {
        return 0;
    }

    TRX_CTRL_1 |= _BV(TX_AUTO_CRC_ON);
    TRX_STATE = 0x09;
    if (!wait_for_radio_state(0x09)) {
        return 0;
    }

    volatile uint8_t *frame_buffer = (volatile uint8_t *)0x180;
    frame_buffer[0] = VEHICLE_FRAME_LENGTH;
    frame_buffer[1] = 0x41;
    frame_buffer[2] = 0x88;
    frame_buffer[3] = (uint8_t)sequence_number;
    frame_buffer[4] = (uint8_t)(VEHICLE_PAN_ID & 0xFF);
    frame_buffer[5] = (uint8_t)(VEHICLE_PAN_ID >> 8);
    frame_buffer[6] = 0xFF;
    frame_buffer[7] = 0xFF;
    frame_buffer[8] = (uint8_t)(VEHICLE_ID & 0xFF);
    frame_buffer[9] = (uint8_t)(VEHICLE_ID >> 8);
    frame_buffer[10] = 'V';
    frame_buffer[11] = '2';
    frame_buffer[12] = 'V';
    frame_buffer[13] = '1';
    frame_buffer[14] = (uint8_t)(sequence_number & 0xFF);
    frame_buffer[15] = (uint8_t)(sequence_number >> 8);

    (void)IRQ_STATUS;
    TRX_STATE = 0x02;
    for (uint32_t timeout = 0; timeout < 100000UL; ++timeout) {
        if (IRQ_STATUS & _BV(TX_END)) {
            return 1;
        }
    }
    return 0;
}

int main(void)
{
    uart_init();
    uart_write("sequence,status\r\n");

    uint16_t sequence_number = 0;
    while (1) {
        ++sequence_number;
        uart_print_uint(sequence_number);
        uart_putc(',');
        uart_write(send_vehicle_beacon(sequence_number) ? "sent\r\n" : "tx_error\r\n");
        _delay_ms(500);
    }
}
