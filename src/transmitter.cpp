#define F_CPU 8000000L                  // CPU clock is 8 MHz, which is used by timing delays and UART baud generation.
#define RF_CHANNEL 20                   // The radio talks on IEEE 802.15.4 channel 20 (the receiver must use the same channel).
#define VEHICLE_PAN_ID 0x1234           // PAN identifier used for this vehicle-proximity network; both nodes must match it.
#define MAC_HEADER_LENGTH 9             // 2-byte frame control + 1-byte sequence + 2-byte PAN + 2-byte destination + 2-byte source = 9 bytes.
#define BEACON_PAYLOAD_LENGTH 6         // Payload contains 4-byte V2V marker + 2-byte sequence number.
#define FCS_LENGTH 2                    // 16-bit CRC/FCS appended automatically by the radio hardware.
#define VEHICLE_FRAME_LENGTH (MAC_HEADER_LENGTH + BEACON_PAYLOAD_LENGTH + FCS_LENGTH) // Total transmit frame length in the radio buffer.

#ifndef VEHICLE_ID                     // Allows overriding the vehicle ID from build flags.
#define VEHICLE_ID 2                   // Default vehicle identifier for this board.
#endif

#include <avr/io.h>                    // AVR hardware register definitions for USART and radio control registers.
#include <stdint.h>                    // Standard integer types like uint8_t and uint16_t.
#include <util/delay.h>                // Delay functions like _delay_ms() used for periodic transmission.

static void uart_init(void)
{
    UCSR1A = 0;                       // Clear USART1 status register to start from a known state.
    UBRR1 = 25;                       // Set 8 MHz / 16 / 19200 baud ~ 25. For 8 MHz AVR, 25 gives ~19200 baud.
    UCSR1B = _BV(TXEN1);              // Enable only transmitter pin for UART1; receiver is not needed here.
    UCSR1C = _BV(UCSZ11) | _BV(UCSZ10); // 8-bit character size: UCSZ1=0b11 -> 8 data bits, no parity, 1 stop bit.
}

static void uart_putc(char value)
{
    while (!(UCSR1A & _BV(UDRE1))) {  // Wait until the UART transmit data register is empty.
    }
    UDR1 = value;                     // Write the byte to UART1; the hardware sends it out on TXD1.
}

static void uart_write(const char *text)
{
    while (*text) {                   // Loop until the string terminator '\0' is reached.
        uart_putc(*text++);          // Send each character in the string one by one.
    }
}

static void uart_print_uint(uint16_t value)
{
    char digits[5];                   // Buffer can hold a 16-bit value up to 65535, which is 5 digits max.
    uint8_t count = 0;                // Number of digits collected in reverse order.

    do {
        digits[count++] = '0' + (value % 10); // Extract the current least-significant decimal digit.
        value /= 10;                  // Remove that digit from the number.
    } while (value != 0);             // Continue until all digits are processed.

    while (count != 0) {              // Print digits in reverse order to show the correct number.
        uart_putc(digits[--count]);  // Decrement count and send the digit back in the right order.
    }
}

static uint8_t wait_for_radio_state(uint8_t expected_state)
{
    uint32_t timeout = 100000UL;      // Give the radio some time to reach the target state; a large number avoids hangs.

    while (timeout-- != 0) {         // Keep polling until timeout expires.
        if ((TRX_STATUS & 0x1F) == expected_state) { // Mask to the lower 5 bits because only state bits matter.
            return 1;                // State reached successfully.
        }
    }
    return 0;                         // Radio never reached the requested state within the timeout.
}

static uint8_t radio_prepare_channel(void)
{
    TRX_STATE = 0x03;                 // Set radio to PLL_ON state: ready to receive/transmit and tune the transceiver.
    if (!wait_for_radio_state(0x08)) { // Wait for state 0x08 = RX_ON or READY? On this chip the expected state is PLL_ON/READY.
        return 0;                    // Radio failed to enter the expected state.
    }

    PHY_CC_CCA = RF_CHANNEL & 0x1F;   // Set the channel number while keeping only the lower 5 bits valid.
    return 1;                         // Channel is now prepared and ready for transmission.
}

static uint8_t send_vehicle_beacon(uint16_t sequence_number)
{
    if (!radio_prepare_channel()) {   // Ensure the radio is in a valid state and on the correct channel.
        return 0;                    // Abort if the radio cannot be configured.
    }

    TRX_CTRL_1 |= _BV(TX_AUTO_CRC_ON); // Turn on automatic CRC generation for transmitted frames.
    TRX_STATE = 0x09;                 // Set TRX_STATE to TX_START: start the transmit sequence.
    if (!wait_for_radio_state(0x09)) { // Wait until the radio indicates TX state is active and ready.
        return 0;                    // Abort if the radio never reaches the expected TX state.
    }

    volatile uint8_t *frame_buffer = (volatile uint8_t *)0x180; // Radio frame buffer sits at address 0x180 in this ATmega256RFR2 memory map.
    frame_buffer[0] = VEHICLE_FRAME_LENGTH; // Byte 0: overall frame length including FCS, as expected by the radio hardware.
    frame_buffer[1] = 0x41;           // Byte 1: frame control field low byte: frame type + version bits for 802.15.4-style frame.
    frame_buffer[2] = 0x88;           // Byte 2: frame control field high byte: indicates data frame with short addresses and ACK request.
    frame_buffer[3] = (uint8_t)sequence_number; // Byte 3: MAC sequence number; this is the packet sequence in the header.
    frame_buffer[4] = (uint8_t)(VEHICLE_PAN_ID & 0xFF); // Byte 4: PAN ID low byte.
    frame_buffer[5] = (uint8_t)(VEHICLE_PAN_ID >> 8); // Byte 5: PAN ID high byte.
    frame_buffer[6] = 0xFF;           // Byte 6: destination address low byte set to broadcast short address 0xFFFF.
    frame_buffer[7] = 0xFF;           // Byte 7: destination address high byte set to broadcast short address 0xFFFF.
    frame_buffer[8] = (uint8_t)(VEHICLE_ID & 0xFF); // Byte 8: source address low byte = vehicle ID.
    frame_buffer[9] = (uint8_t)(VEHICLE_ID >> 8); // Byte 9: source address high byte = vehicle ID.
    frame_buffer[10] = 'V';           // Byte 10: payload marker first char 'V' for vehicle beacon.
    frame_buffer[11] = '2';           // Byte 11: payload marker second char '2'.
    frame_buffer[12] = 'V';           // Byte 12: payload marker third char 'V'.
    frame_buffer[13] = '1';           // Byte 13: payload marker fourth char '1'; together this makes the signature "V2V1".
    frame_buffer[14] = (uint8_t)(sequence_number & 0xFF); // Byte 14: low byte of the beacon sequence counter.
    frame_buffer[15] = (uint8_t)(sequence_number >> 8); // Byte 15: high byte of the beacon sequence counter.

    (void)IRQ_STATUS;                 // Read IRQ_STATUS once to clear any stale interrupt flag before starting the TX.
    TRX_STATE = 0x02;                 // Set radio to RX_ON or send to TX? On this firmware it transitions back to RX_ON after the write; TX_END will occur afterward.
    for (uint32_t timeout = 0; timeout < 100000UL; ++timeout) { // Wait for hardware completion of the TX cycle.
        if (IRQ_STATUS & _BV(TX_END)) { // Check whether the radio has asserted the TX end interrupt.
            return 1;                // Transmission was successful.
        }
    }
    return 0;                         // No TX_END event arrived before timeout, so transmission failed.
}

int main(void)
{
    uart_init();                      // Initialize UART1 so the board can print status text to the serial terminal.
    uart_write("sequence,status\r\n"); // Print the CSV header used by the receiver/monitor to see the packet counter and result.

    uint16_t sequence_number = 0;     // Keeps a rolling counter that increments for every beacon sent.
    while (1) {                       // Continue forever transmitting beacons with a fixed period.
        ++sequence_number;            // Advance the beacon sequence for every packet.
        uart_print_uint(sequence_number); // Print the current sequence number to the serial monitor.
        uart_putc(',');              // Separate the sequence and status with a comma for simple CSV output.
        uart_write(send_vehicle_beacon(sequence_number) ? "sent\r\n" : "tx_error\r\n"); // Transmit the frame and immediately print the result status.
        _delay_ms(500);               // Pause 500 ms to create a periodic beacon rate of about 2 packets per second.
    }
}
