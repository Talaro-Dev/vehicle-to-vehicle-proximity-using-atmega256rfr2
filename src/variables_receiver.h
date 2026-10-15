#ifndef VARIABLES_RECEIVER_H
#define VARIABLES_RECEIVER_H

#include <stdint.h>

// parameters for receiver
#define F_CPU 8000000L //8 MHz
#define RF_CHANNEL 20 // radio channel 20
#define VEHICLE_PAN_ID 0x1234 // Network_ID for communication between receiver transmitter

#define RSSI_FILTER_RAW 0   
#define RSSI_FILTER_MEDIAN 1                 
#define RSSI_FILTER_MOVING_AVERAGE 2        
#define RSSI_FILTER_KALMAN 3                 
#define RSSI_FILTER_HYBRID_GAUSSIAN_KALMAN 4 
#define RSSI_FILTER_SNR_ADAPTIVE_KALMAN 5    
#ifndef RSSI_FILTER_MODE
#define RSSI_FILTER_MODE RSSI_FILTER_SNR_ADAPTIVE_KALMAN
#endif


// Packet format [header 9 bytes, payload 6 bytes, FCS 2 bytes]
#define MAC_HEADER_LENGTH 9
#define BEACON_PAYLOAD_LENGTH 6
#define FCS_LENGTH 2
#define VEHICLE_FRAME_LENGTH (MAC_HEADER_LENGTH + BEACON_PAYLOAD_LENGTH + FCS_LENGTH)

#define MAX_TRACKED_VEHICLES 4

#define RADIO_CMD_FORCE_TRX_OFF 0x03
#define RADIO_CMD_RX_ON 0x06
#define RADIO_STATE_MASK 0x1F // only the low 5 bits of TRX_STATUS hold the state

#define FRAME_BUFFER_ADDRESS 0x180 // store received frames 

typedef struct {
    int16_t samples[3];
    uint8_t sample_count;
    uint8_t next_sample;
    int32_t estimate_value;
    uint32_t covariance_value;
    uint8_t kalman_initialized;
} rssifilterstate;

typedef struct {
    uint16_t vehicle_id;
    uint32_t last_packet;
    uint8_t used;
    rssifilterstate filter;
} Vehicle;

typedef struct {
    uint8_t length;
    uint8_t crc_ok;
    uint8_t rssi_raw;
    int16_t rssi_dbm;
    const char *status;
    uint8_t is_beacon;
    uint16_t vehicle_id;
    uint16_t sequence;
    uint8_t lqi;
    uint8_t noise_valid;
    uint8_t ed_level;
    int16_t snr;
    int16_t rssi_filtered;
} packetinfo;

#endif

