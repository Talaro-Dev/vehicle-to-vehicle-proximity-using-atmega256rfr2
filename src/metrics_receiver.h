#ifndef METRICS_RECEIVER_H
#define METRICS_RECEIVER_H

#include <stdint.h>
#include "uart_receiver.h"
#include "variables_receiver.h"

static void print_csv_line(const packetinfo *info, uint32_t packet_number)
{
    uart_print_uint(packet_number);
    uart_putc(',');
    uart_print_uint(RF_CHANNEL);
    uart_putc(',');
    uart_print_uint(info->length);
    uart_putc(',');
    uart_print_uint(info->crc_ok);
    uart_putc(',');
    uart_write(info->status);
    uart_putc(',');

    if (info->is_beacon)
    {
        uart_print_uint(info->vehicle_id);
    }
    uart_putc(',');
    if (info->is_beacon)
    {
        uart_print_uint(info->sequence);
    }
    uart_putc(',');
    uart_print_int(info->rssi_dbm);
    uart_putc(',');
    if(info->is_beacon)
    {
        uart_print_int(info->rssi_filtered);
    }
    uart_putc(',');
    uart_print_uint(info->rssi_raw);
    uart_putc(',');

    if(info->is_beacon)
    {
        uart_print_int(info->lqi);
    }
    uart_putc(',');

    if(info->noise_valid)
    {
        uart_print_int(info->ed_level);
    }
    uart_putc(',');
    if(info->noise_valid)
    {
        uart_print_int(info->snr);
    }
    uart_putc(',');

    uart_print_int(RSSI_FILTER_MODE);
    uart_write("\r\n");
}

#endif