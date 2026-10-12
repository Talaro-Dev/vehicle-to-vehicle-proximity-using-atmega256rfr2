#define F_CPU 8000000L // Tell AVR delay routines and baud calculations that the CPU runs at 8 MHz.
#define RF_CHANNEL 20 // Listen on IEEE 802.15.4 channel 20; the transmitter must use this too.
#define VEHICLE_PAN_ID 0x1234 // Network identifier expected in the beacon's 802.15.4 MAC header.
#define MAC_HEADER_LENGTH 9 // Header size for this beacon: FCF, sequence, destination PAN/short address, source short address.
#define BEACON_PAYLOAD_LENGTH 6 // Application payload: four marker bytes plus a two-byte beacon sequence.
#define FCS_LENGTH 2 // IEEE 802.15.4 frame check sequence (CRC) occupies two bytes on air.
#define VEHICLE_FRAME_LENGTH (MAC_HEADER_LENGTH + BEACON_PAYLOAD_LENGTH + FCS_LENGTH) // Only accept this beacon's expected frame length.
#define MAX_TRACKED_VEHICLES 4 // Keep independent filter history for up to four vehicle IDs.
#define RSSI_FILTER_RAW 0
#define RSSI_FILTER_MEDIAN3 1
#define RSSI_FILTER_MOVING_AVERAGE3 2
#define RSSI_FILTER_KALMAN 3
#define RSSI_FILTER_HYBRID_GAUSSIAN_KALMAN 4
#define RSSI_FILTER_SNR_ADAPTIVE_KALMAN 5
#ifndef RSSI_FILTER_MODE
#define RSSI_FILTER_MODE RSSI_FILTER_SNR_ADAPTIVE_KALMAN // Select another mode here to compare filters.
#endif

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

// Configure USART1, which is connected to the board's EDBG virtual COM port.
static void uart_init(void)
{
    UCSR1A = 0; // Clear UART mode bits, selecting normal-speed asynchronous operation.
    UBRR1 = 12; // At 8 MHz this gives about 38400 baud, fast enough for 50 beacon reports per second.
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

typedef struct {
    int16_t samples[3];
    uint8_t sample_count;
    uint8_t next_sample;
    int32_t estimate_q8;
    uint32_t covariance_q8;
    uint8_t kalman_initialized;
} RssiFilterState;

typedef struct {
    uint16_t vehicle_id;
    uint32_t last_packet;
    uint8_t used;
    RssiFilterState filter;
} VehicleFilterState;

static VehicleFilterState vehicle_filters[MAX_TRACKED_VEHICLES];

static RssiFilterState *get_vehicle_filter(uint16_t vehicle_id, uint32_t packet_number)
{
    int8_t selected = -1;
    uint32_t oldest_packet = 0xFFFFFFFFUL;

    for (uint8_t index = 0; index < MAX_TRACKED_VEHICLES; ++index) {
        if (vehicle_filters[index].used && vehicle_filters[index].vehicle_id == vehicle_id) {
            selected = (int8_t)index;
            break;
        }
        if (!vehicle_filters[index].used) {
            selected = (int8_t)index;
            break;
        }
        if (vehicle_filters[index].last_packet < oldest_packet) {
            oldest_packet = vehicle_filters[index].last_packet;
            selected = (int8_t)index;
        }
    }

    VehicleFilterState *vehicle = &vehicle_filters[(uint8_t)selected];
    if (!vehicle->used || vehicle->vehicle_id != vehicle_id) {
        vehicle->used = 1;
        vehicle->vehicle_id = vehicle_id;
        vehicle->filter.sample_count = 0;
        vehicle->filter.next_sample = 0;
        vehicle->filter.estimate_q8 = 0;
        vehicle->filter.covariance_q8 = 0;
        vehicle->filter.kalman_initialized = 0;
    }
    vehicle->last_packet = packet_number;
    return &vehicle->filter;
}

static void remember_rssi(RssiFilterState *state, int16_t rssi_dbm)
{
    state->samples[state->next_sample] = rssi_dbm;
    state->next_sample = (state->next_sample + 1) % 3;
    if (state->sample_count < 3) {
        ++state->sample_count;
    }
}

static int16_t median_rssi(const RssiFilterState *state)
{
    int16_t values[3];
    for (uint8_t index = 0; index < state->sample_count; ++index) {
        values[index] = state->samples[index];
    }
    for (uint8_t index = 0; index < state->sample_count; ++index) {
        for (uint8_t next = index + 1; next < state->sample_count; ++next) {
            if (values[next] < values[index]) {
                int16_t temporary = values[index];
                values[index] = values[next];
                values[next] = temporary;
            }
        }
    }
    return values[state->sample_count / 2];
}

static int16_t moving_average_rssi(const RssiFilterState *state)
{
    int32_t sum = 0;
    for (uint8_t index = 0; index < state->sample_count; ++index) {
        sum += state->samples[index];
    }
    return (int16_t)(sum / state->sample_count);
}

static uint16_t adaptive_measurement_variance(int16_t snr_db, uint8_t lqi, uint8_t noise_valid)
{
    if (!noise_valid || snr_db < 3 || lqi < 64) {
        return 100; // Poor or unknown link quality: trust this RSSI measurement less.
    }
    if (snr_db < 8 || lqi < 128) {
        return 49;
    }
    if (snr_db < 15 || lqi < 192) {
        return 25;
    }
    return 9; // Good SNR and LQI: trust the measurement more.
}

static uint8_t gaussian_likelihood_q8(uint8_t residual_db)
{
    static const uint8_t likelihood[] = {255, 248, 226, 199, 155, 120, 83, 54, 35, 21, 11, 6, 3, 1};
    if (residual_db >= sizeof(likelihood)) {
        return 1;
    }
    return likelihood[residual_db];
}

static int16_t kalman_rssi(RssiFilterState *state, int16_t measurement_dbm,
                           uint16_t measurement_variance, uint8_t gaussian_hybrid)
{
    if (!state->kalman_initialized) {
        state->estimate_q8 = (int32_t)measurement_dbm * 256;
        state->covariance_q8 = (uint32_t)measurement_variance * 256;
        state->kalman_initialized = 1;
        return measurement_dbm;
    }

    if (gaussian_hybrid) {
        int32_t residual = measurement_dbm - state->estimate_q8 / 256;
        if (residual < 0) {
            residual = -residual;
        }
        uint8_t likelihood = gaussian_likelihood_q8((uint8_t)residual);
        measurement_variance = (measurement_variance * 256UL) / likelihood;
        if (measurement_variance > 1000) {
            measurement_variance = 1000;
        }
    }

    uint32_t prior_covariance_q8 = state->covariance_q8 + 1024; // Q=4 dB^2 process noise per 50 Hz packet.
    uint32_t measurement_covariance_q8 = (uint32_t)measurement_variance * 256;
    uint32_t gain_q15 = (prior_covariance_q8 << 15) /
                        (prior_covariance_q8 + measurement_covariance_q8);
    int32_t residual_q8 = (int32_t)measurement_dbm * 256 - state->estimate_q8;
    state->estimate_q8 += (residual_q8 * (int32_t)gain_q15) >> 15;
    state->covariance_q8 = ((32768UL - gain_q15) * prior_covariance_q8) >> 15;
    return (int16_t)(state->estimate_q8 / 256);
}

static int16_t filter_rssi(RssiFilterState *state, int16_t raw_rssi_dbm,
                           int16_t snr_db, uint8_t lqi, uint8_t noise_valid)
{
    remember_rssi(state, raw_rssi_dbm);

    switch (RSSI_FILTER_MODE) {
    case RSSI_FILTER_RAW:
        return raw_rssi_dbm;
    case RSSI_FILTER_MEDIAN3:
        return median_rssi(state);
    case RSSI_FILTER_MOVING_AVERAGE3:
        return moving_average_rssi(state);
    case RSSI_FILTER_KALMAN:
        return kalman_rssi(state, raw_rssi_dbm, 9, 0);
    case RSSI_FILTER_HYBRID_GAUSSIAN_KALMAN:
        return kalman_rssi(state, raw_rssi_dbm, 9, 1);
    case RSSI_FILTER_SNR_ADAPTIVE_KALMAN:
        return kalman_rssi(state, raw_rssi_dbm,
                           adaptive_measurement_variance(snr_db, lqi, noise_valid), 0);
    default:
        return raw_rssi_dbm;
    }
}

// Wait for the radio's status register to report a requested transceiver state.ß
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

// Take a short energy-detect sample after a frame to estimate current channel noise/interference.
static uint8_t measure_channel_energy(uint8_t *energy_level)
{
    PHY_CC_CCA = RF_CHANNEL & 0x1F; // Channel 20 with CCA mode 00, the radio's energy-detect mode.
    PHY_CC_CCA |= _BV(CCA_REQUEST); // Start the CCA/energy-detect operation.

    for (uint16_t timeout = 0; timeout < 300; ++timeout) { // Bound the wait to about 3 ms at 10 us per check.
        if (TRX_STATUS & _BV(CCA_DONE)) { // CCA_DONE is set when the radio has finished measuring.
            *energy_level = PHY_ED_LEVEL; // Return the measured ED code to the caller.
            return 1;
        }
        _delay_us(10); // Give the radio time to finish before checking status again.
    }
    return 0; // Report that no valid noise/energy sample was available.
}

// Called after RX_END. Log every frame so rejected packets can be diagnosed from the serial output.
static void print_received_beacon(uint32_t packet_number)
{
    volatile uint8_t *frame_buffer = (volatile uint8_t *)0x180; // ATmega256RFR2 maps its radio frame buffer at data address 0x180.
    uint8_t frame_length = TST_RX_LENGTH; // The RX buffer starts at the first PSDU/MAC byte; length is reported separately.
    uint8_t phy_rssi = PHY_RSSI; // Read RSSI and receive-status bits captured for this frame.
    uint8_t raw_rssi = phy_rssi & 0x1F; // Bits 0-4 contain the raw five-bit RSSI code.
    int16_t rssi_dbm = -91 + 3 * raw_rssi; // Convert the nominal RSSI code to dBm in 3 dB steps.
    uint8_t crc_valid = (phy_rssi & _BV(RX_CRC_VALID)) != 0; // Bit 7 is set when the hardware accepted the frame CRC.
    uint16_t vehicle_id = 0;
    uint16_t beacon_sequence = 0;
    const char *status = "bad_crc";
    uint8_t beacon_match = 0;
    uint8_t lqi = 0;
    uint8_t ed_level = 0;
    uint8_t noise_valid = 0;
    int16_t snr_est_db = 0;
    int16_t filtered_rssi_dbm = rssi_dbm;

    if (crc_valid) {
        if (frame_length == 0 || frame_length > 127) {
            status = "bad_length";
        } else if (frame_length != VEHICLE_FRAME_LENGTH &&
                   frame_length != VEHICLE_FRAME_LENGTH - FCS_LENGTH) {
            status = "length_mismatch";
        } else {
            uint16_t frame_control = frame_buffer[0] | ((uint16_t)frame_buffer[1] << 8);
            uint16_t destination_pan = frame_buffer[3] | ((uint16_t)frame_buffer[4] << 8);
            uint16_t destination_address = frame_buffer[5] | ((uint16_t)frame_buffer[6] << 8);
            volatile uint8_t *payload = frame_buffer + MAC_HEADER_LENGTH;

            if (frame_control == 0x8841 && destination_pan == VEHICLE_PAN_ID &&
                destination_address == 0xFFFF && payload[0] == 'V' &&
                payload[1] == '2' && payload[2] == 'V' && payload[3] == '1') {
                vehicle_id = frame_buffer[7] | ((uint16_t)frame_buffer[8] << 8);
                beacon_sequence = payload[4] | ((uint16_t)payload[5] << 8);
                beacon_match = 1;
                status = "beacon";
                lqi = frame_buffer[frame_length]; // RFR2 stores the received frame's LQI immediately after the PSDU.
                noise_valid = measure_channel_energy(&ed_level); // Sample channel energy after reading packet RSSI/LQI.
                if (noise_valid) {
                    int16_t noise_dbm = -91 + ed_level; // ED level is nominally about 1 dB per step from -91 dBm.
                    snr_est_db = rssi_dbm - noise_dbm; // Approximate SNR as packet RSSI minus measured channel energy.
                }
                RssiFilterState *filter = get_vehicle_filter(vehicle_id, packet_number);
                filtered_rssi_dbm = filter_rssi(filter, rssi_dbm, snr_est_db, lqi, noise_valid);
            } else {
                status = "other_frame";
            }
        }
    }

    uart_print_uint(packet_number); // Count of RX_END events; unrelated frames may make this number skip.
    uart_putc(',');
    uart_print_uint(RF_CHANNEL); // Fixed channel used by both boards.
    uart_putc(',');
    uart_print_uint(frame_length); // Length reported by the radio for the received PSDU.
    uart_putc(',');
    uart_print_uint(crc_valid); // 1 means hardware CRC passed.
    uart_putc(',');
    uart_write(status); // Beacon or the reason a received frame was rejected.
    uart_putc(',');
    if (beacon_match) uart_print_uint(vehicle_id); // Source short address, empty for other frames.
    uart_putc(',');
    if (beacon_match) uart_print_uint(beacon_sequence); // Payload sequence counter, empty for other frames.
    uart_putc(',');
    uart_print_int(rssi_dbm); // Unfiltered packet RSSI estimate.
    uart_putc(',');
    if (beacon_match) uart_print_int(filtered_rssi_dbm); // Selected filter output; empty for non-beacon frames.
    uart_putc(',');
    uart_print_uint(raw_rssi); // Five-bit hardware RSSI value.
    uart_putc(',');
    if (beacon_match) uart_print_uint(lqi); // Link-quality byte appended after the received PSDU.
    uart_putc(',');
    if (noise_valid) uart_print_uint(ed_level); // Raw energy-detect sample used to estimate channel noise.
    uart_putc(',');
    if (noise_valid) uart_print_int(snr_est_db); // Estimated SNR = packet RSSI minus ED-derived noise level.
    uart_putc(',');
    uart_print_uint(RSSI_FILTER_MODE); // Numeric filter mode from the RSSI_FILTER_* definitions.
    uart_write("\r\n");
}

int main(void) {
    uart_init(); // Start serial output before initializing the radio.
    uart_write("packet,channel,length,crc_valid,status,vehicle_id,sequence,raw_dbm,filtered_dbm,rssi_raw,lqi,ed,snr_est_db,filter_mode\r\n"); // CSV output includes raw and filtered link measurements.

    if (!radio_start_receiver()) { // Configure channel 20 and attempt to enter RX_ON.
        uart_write("radio_start_error\r\n"); // Report that the transceiver did not reach its receive state.
        while (1) { // Stay here instead of pretending reception is working.
        }
    }

    uint32_t packet_number = 0; // Counts completed receive events; unrelated frames can make this number skip in the CSV.
    while (1) { // Keep checking for completed radio frames indefinitely.
        uint8_t irq_status = IRQ_STATUS; // Snapshot radio event flags once for this loop iteration.
        if (irq_status & _BV(RX_END)) { // RX_END means the radio has finished receiving a frame.
            IRQ_STATUS = _BV(RX_END); // Clear the latched RX_END flag so the same buffer is not logged repeatedly.
            print_received_beacon(++packet_number); // Count the event, validate/filter its frame, then print matching beacons.
        }
    }
}