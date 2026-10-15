#include <avr/io.h>
#include <stdint.h>
#include "variables_transmitter.h"
#include <util/delay.h>
#include "uart_transmitter.h"


static uint8_t wait_for_radio_state(uint8_t expected_state)
{
    uint32_t timeout = 100000UL;

    while (timeout > 0) {
        if ((TRX_STATUS & RADIO_STATE_MASK) == expected_state) {
            return 1;
        }
        timeout--;
    }
    return 0;
}

static uint8_t radio_prepare_channel(void)
{
    TRX_STATE = RADIO_CMD_FORCE_TRX_OFF;
    if (!wait_for_radio_state(RADIO_STATE_TRX_OFF)) {
        return 0;
    }

    PHY_CC_CCA = RF_CHANNEL & 0x1F;
    return 1;
}

static void fill_beacon_frame(uint16_t sequence_number)
{
    volatile uint8_t *buffer = (volatile uint8_t *)FRAME_BUFFER_ADDRESS;
    uint8_t i = 0;

    buffer[i++] = VEHICLE_FRAME_LENGTH;

    buffer[i++] = 0x41;
    buffer[i++] = 0x88;
    buffer[i++] = (uint8_t)sequence_number;
    buffer[i++] = (uint8_t)(VEHICLE_PAN_ID & 0xFF);
    buffer[i++] = (uint8_t)(VEHICLE_PAN_ID >> 8);
    buffer[i++] = 0xFF;
    buffer[i++] = 0xFF;
    buffer[i++] = (uint8_t)(VEHICLE_ID & 0xFF);
    buffer[i++] = (uint8_t)(VEHICLE_ID >> 8);

    buffer[i++] = 'V';
    buffer[i++] = '2';
    buffer[i++] = 'V';
    buffer[i++] = '1';
    buffer[i++] = (uint8_t)(sequence_number & 0xFF);
    buffer[i++] = (uint8_t)(sequence_number >> 8);
}

static uint8_t send_vehicle_beacon(uint16_t sequence_number)
{
    if (!radio_prepare_channel()) {
        return 0;
    }

    TRX_CTRL_1 |= _BV(TX_AUTO_CRC_ON);

    TRX_STATE = RADIO_CMD_PLL_ON;
    if (!wait_for_radio_state(RADIO_STATE_PLL_ON)) {
        return 0;
    }

    fill_beacon_frame(sequence_number);

    IRQ_STATUS = _BV(TX_END);
    TRX_STATE = RADIO_CMD_TX_START;

    for (uint32_t tries = 0; tries < 100000UL; tries++) {
        if (IRQ_STATUS & _BV(TX_END)) {
            IRQ_STATUS = _BV(TX_END);
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
        sequence_number++;

        uart_print_uint(sequence_number);
        uart_putc(',');

        if (send_vehicle_beacon(sequence_number)) {
            uart_write("sent\r\n");
        } else {
            uart_write("tx_error\r\n");
        }

        _delay_ms(BEACON_PERIOD_MS);
    }
}
