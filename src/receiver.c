#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include "uart_receiver.h"
#include "filters_receiver.h"
#include "variables_receiver.h"
#include "metrics_receiver.h"

#define RADIO_STATE_TRX_OFF 0x08
#define RADIO_STATE_RX_ON 0x06

static Vehicle vehicle_filters[MAX_TRACKED_VEHICLES];

static uint8_t wait_for_radio_state(uint8_t expected_state)
{
    uint32_t timeout = 100000;

    while (timeout > 0 )
    {
        if ((TRX_STATUS & RADIO_STATE_MASK) == expected_state)
        {
            return 1;
        }
        timeout--;
    }
    return 0;
}

static uint8_t radio_prepare_channel(void)
{
    TRX_STATE = RADIO_CMD_FORCE_TRX_OFF;
    if (!wait_for_radio_state(RADIO_STATE_TRX_OFF))
    {
        return 0;
    }

    PHY_CC_CCA = RF_CHANNEL & 0x1F; //channel number lives in the low 5 bits
    return 1;
}

static uint8_t radio_init(void)
{
    if(!radio_prepare_channel())
    {
        return 0;
    }

    IRQ_MASK = _BV(RX_END_EN);
    (void)IRQ_STATUS;
    TRX_STATE = RADIO_CMD_RX_ON;
    return wait_for_radio_state(RADIO_STATE_RX_ON);
}

static uint8_t measure_channel_energy(uint8_t *energylevel)
{
    PHY_CC_CCA = RF_CHANNEL & 0x1F;
    PHY_CC_CCA |= _BV(CCA_REQUEST);

    for(uint16_t tries = 0; tries < 300; tries++)
    {
        if(TRX_STATUS & _BV(CCA_DONE))
        {
            *energylevel = PHY_ED_LEVEL;
            return 1;
        }
        _delay_us(10);
    }
    return 0;
}

static void check_packet(packetinfo *info, volatile uint8_t *buffer)
{
    if (!info->crc_ok){
        info->status = "bad_crc";
        return;
    }
    if (info->length ==0 || info-> length > 127)
    {
        info->status = "bad_length";
        return;
    }
    if(info->length != VEHICLE_FRAME_LENGTH && info->length != VEHICLE_FRAME_LENGTH - FCS_LENGTH)
    {
        info->status = "length_mismatch";
        return;
    }

    uint16_t frame_control = read_u16(buffer, 0);
    uint16_t destination_pan = read_u16(buffer, 3);
    uint16_t destination_address = read_u16(buffer, 5);
    volatile uint8_t *payload = buffer + MAC_HEADER_LENGTH;

    if (frame_control == 0x8841 && destination_pan == VEHICLE_PAN_ID &&
    destination_address == 0xFFFF &&
    payload[0] == 'V' && payload[1] == '2' && payload[2] == 'V' && payload[3] == '1')
    {
        info->is_beacon = 1;
        info->status = "beacon";
        info->vehicle_id = read_u16(buffer, 7);
        info->sequence = read_u16(payload, 4);
        info->lqi = buffer[info->length];

    }
    else
    {
        info->status = "other_frame";
    }
}

static void handle_received_packet(uint32_t packet_number)
{
    volatile uint8_t *buffer = (volatile uint8_t *)FRAME_BUFFER_ADDRESS;
    uint8_t phy_rssi = PHY_RSSI;
    packetinfo info = {0};

    info.length = TST_RX_LENGTH;
    info.rssi_raw = phy_rssi & 0x1F;
    info.rssi_dbm = -91 + 3 * info.rssi_raw;
    info.crc_ok = (phy_rssi & _BV(RX_CRC_VALID)) != 0;
    info.rssi_filtered = info.rssi_dbm;
    info.status = "bad_crc";

    check_packet(&info, buffer);
    
    if (info.is_beacon)
    {
        info.noise_valid = measure_channel_energy(&info.ed_level);
        if (info.noise_valid)
        {
            int16_t noise_dbm = -91 + info.ed_level;
            info.snr = info.rssi_dbm - noise_dbm;
        }

        rssifilterstate *filter = get_vehicle_filter(info.vehicle_id, packet_number, MAX_TRACKED_VEHICLES, vehicle_filters);
        info.rssi_filtered = filter_rssi(filter, info.rssi_dbm, info.snr, info.lqi, info.noise_valid);
    }
    print_csv_line(&info, packet_number);
}

int main(void)
{
    uart_init();
    uart_write("packet,channel,length,crc_valid,status,vehicle_id,sequence,raw_dbm,filtered_dbm,rssi_raw,lqi,ed,snr_est_db,filter_mode\r\n");

    if (!radio_init())
    {
        uart_write("Failed to start radio receiver\r\n");
        while(1)
        {

        }
    }

    uint32_t packet_number = 0;
    while (1)
    {
        if (IRQ_STATUS & _BV(RX_END))
        {
            IRQ_STATUS = _BV(RX_END);
            packet_number++;
            handle_received_packet(packet_number);
        }
    }
}

