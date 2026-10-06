#define F_CPU 8000000L // Tell AVR delay routines and baud calculations that the CPU runs at 8 MHz.
#define RF_CHANNEL 20 // Listen on IEEE 802.15.4 channel 20; the transmitter must use this too.
#define VEHICLE_PAN_ID 0x1234 // Network identifier expected in the beacon's 802.15.4 MAC header.
#define MAC_HEADER_LENGTH 9 // Header size for this beacon: FCF, sequence, destination PAN/short address, source short address.
#define BEACON_PAYLOAD_LENGTH 6 // Application payload: four marker bytes plus a two-byte beacon sequence.
#define FCS_LENGTH 2 // IEEE 802.15.4 frame check sequence (CRC) occupies two bytes on air.
#define VEHICLE_FRAME_LENGTH (MAC_HEADER_LENGTH + BEACON_PAYLOAD_LENGTH + FCS_LENGTH) // Only accept this beacon's expected frame length.

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

// Configure USART1, which is connected to the board's EDBG virtual COM port.
static void uart_init(void)
{
    UCSR1A = 0; // Clear UART mode bits, selecting normal-speed asynchronous operation.
    UBRR1 = 25; // Divide the 8 MHz clock to approximately 19200 baud.
    UCSR1B = _BV(TXEN1); // Enable UART transmission; the receiver and UART interrupts stay disabled.
    UCSR1C = _BV(UCSZ11) | _BV(UCSZ10); // Select 8 data bits, no parity, and one stop bit (8-N-1).
}

// Send one character over USART1.
static void uart_putc(char c)
{
    while (!(UCSR1A & _BV(UDRE1))) { // Wait until the UART data register can accept another character.
    }
    UDR1 = c; // Writing the character starts its transmission.

}

// Send a zero-terminated C string over USART1.
static void uart_write(const char *text)
{
    while (*text) { // Continue until the string's terminating zero byte.
        uart_putc(*text++); // Send this character, then advance to the next one.
    }
}

// Convert an unsigned integer to decimal text and send it over UART.
static void uart_print_uint(uint32_t value)
{
    char digits[10]; // A 32-bit unsigned integer needs at most ten decimal digits.
    uint8_t i = 0; // Number of digits currently stored in the array.

    do {
        digits[i++] = (value % 10) + '0'; // Extract the last decimal digit and turn it into an ASCII character.
        value /= 10; // Remove the digit just extracted.
    } while (value > 0); // Digits are collected least-significant first.

    while (i != 0) { // Send the saved digits in the opposite order to print normally.
        uart_putc(digits[--i]); // Decrement first, then send that array element.
    }
}

// Print a signed integer, including a minus sign when it is negative.
static void uart_print_int(int16_t value)
{
    if (value < 0) {
        uart_putc('-'); // CSV dBm values are normally negative.
        value = -value; // Convert the magnitude to positive for the unsigned printer.
    }
    uart_print_uint((uint16_t)value); // Reuse the decimal unsigned-integer formatter.
}

// Wait for the radio's status register to report a requested transceiver state.
static uint8_t wait_for_radio_state(uint8_t expected_state)
{
    uint32_t timeout = 100000UL; // Bound the wait so a hardware/state problem cannot hang startup forever.

    while (timeout-- != 0) {
        if ((TRX_STATUS & 0x1F) == expected_state) { // The low five status bits encode the current radio state.
            return 1; // Nonzero means the requested state was reached.
        }
    }
    return 0; // Zero means the state did not appear before the timeout.
}

// Stop the radio and set its channel; leave it off for the caller to choose RX mode.
static uint8_t radio_prepare_channel(void)
{
    TRX_STATE = 0x03; // 0x03 is the FORCE_TRX_OFF command written to TRX_STATE.
    if (!wait_for_radio_state(0x08)) { // 0x08 is the TRX_OFF state read back from TRX_STATUS.
        return 0; // Do not configure the channel unless the radio is safely off.
    }

    PHY_CC_CCA = RF_CHANNEL & 0x1F; // Put the channel number in the channel field of PHY_CC_CCA.
    return 1; // The radio is off and configured for the requested channel.
}

// Enable receive-end reporting, clear any old radio event, then enter RX_ON.
static uint8_t radio_start_receiver(void)
{
    if (!radio_prepare_channel()) { // Prepare the channel before changing to receive mode.
        return 0; // Propagate the radio-state timeout to main().
    }

    IRQ_MASK = _BV(RX_END_EN); // Enable the radio's RX_END event for a completed frame.
    (void)IRQ_STATUS; // Read/clear stale event flags before starting a new receive session.
    TRX_STATE = 0x06; // 0x06 is the RX_ON command.
    return wait_for_radio_state(0x06); // Confirm RX_ON; return whether startup succeeded.
}

// Called after RX_END. Ignore other frames; print only valid V2V1 vehicle beacons.
static void print_received_beacon(uint32_t packet_number)
{
    volatile uint8_t *frame_buffer = (volatile uint8_t *)0x180; // ATmega256RFR2 maps its radio frame buffer at data address 0x180.
    uint8_t frame_length = frame_buffer[0]; // Byte zero is the received PHY frame-length field.
    uint8_t phy_rssi = PHY_RSSI; // Read RSSI and receive-status bits captured for this frame.
    uint8_t raw_rssi = phy_rssi & 0x1F; // Bits 0-4 contain the raw five-bit RSSI code.
    int16_t rssi_dbm = -91 + 3 * raw_rssi; // Convert the nominal RSSI code to dBm in 3 dB steps.
    uint8_t crc_valid = (phy_rssi & _BV(RX_CRC_VALID)) != 0; // Bit 7 is set when the hardware accepted the frame CRC.

    if (!crc_valid || frame_length != VEHICLE_FRAME_LENGTH) { // Reject corrupted frames and frames with an unexpected size.
        return; // No output means this was not an acceptable vehicle beacon.
    }

    uint16_t frame_control = frame_buffer[1] | ((uint16_t)frame_buffer[2] << 8); // Assemble the little-endian 802.15.4 frame-control field.
    uint16_t destination_pan = frame_buffer[4] | ((uint16_t)frame_buffer[5] << 8); // Assemble the destination PAN ID from its two bytes.
    uint16_t destination_address = frame_buffer[6] | ((uint16_t)frame_buffer[7] << 8); // Assemble the destination short address.
    volatile uint8_t *payload = frame_buffer + 1 + MAC_HEADER_LENGTH; // Skip the length byte and nine-byte MAC header to reach app data.

    if (frame_control != 0x8841 || destination_pan != VEHICLE_PAN_ID ||
        destination_address != 0xFFFF || payload[0] != 'V' ||
        payload[1] != '2' || payload[2] != 'V' || payload[3] != '1') { // Require our data-frame layout, PAN, broadcast address, and V2V1 marker.
        return; // Ignore unrelated valid 802.15.4 traffic.
    }

    uint16_t vehicle_id = frame_buffer[8] | ((uint16_t)frame_buffer[9] << 8); // The source short address identifies the transmitting vehicle.
    uint16_t beacon_sequence = payload[4] | ((uint16_t)payload[5] << 8); // The payload's final two bytes are the sender's sequence counter.

    uart_print_uint(packet_number); // Print the receiver's count of RX_END events seen so far.
    uart_putc(','); // Separate CSV columns with commas.
    uart_print_uint(RF_CHANNEL); // Report the fixed channel used by this receiver.
    uart_putc(','); // Begin the vehicle-ID column.
    uart_print_uint(vehicle_id); // Print the sender's short address as its vehicle ID.
    uart_putc(','); // Begin the beacon sequence column.
    uart_print_uint(beacon_sequence); // Print the sequence number from the application payload.
    uart_putc(','); // Begin the estimated RSSI column.
    uart_print_int(rssi_dbm); // Print nominal RSSI in dBm; this estimate has 3 dB resolution.
    uart_putc(','); // Begin the raw RSSI column.
    uart_print_uint(raw_rssi); // Preserve the unconverted hardware reading for comparison/calibration.
    uart_write(",ok\r\n"); // End the CSV row and mark the filtered beacon valid.
}

int main(void) {
    uart_init(); // Start serial output before initializing the radio.
    uart_write("packet,channel,vehicle_id,sequence,rssi_dbm,rssi_raw,status\r\n"); // Print the CSV column names once at startup.

    if (!radio_start_receiver()) { // Configure channel 20 and attempt to enter RX_ON.
        uart_write("radio_start_error\r\n"); // Report that the transceiver did not reach its receive state.
        while (1) { // Stay here instead of pretending reception is working.
        }
    }

    uint32_t packet_number = 0; // Counts completed receive events; unrelated frames can make this number skip in the CSV.
    while (1) { // Keep checking for completed radio frames indefinitely.
        if (IRQ_STATUS & _BV(RX_END)) { // RX_END means the radio has finished receiving a frame.
            print_received_beacon(++packet_number); // Count the event, validate/filter its frame, then print matching beacons.
        }
    }
}