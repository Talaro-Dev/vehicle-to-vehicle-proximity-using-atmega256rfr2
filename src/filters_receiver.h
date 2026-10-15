#ifndef FILTERS_RECEIVER_H
#define FILTERS_RECEIVER_H

#include <stdint.h>
#include "variables_receiver.h"

static rssifilterstate *get_vehicle_filter(uint16_t vehicle_id, uint32_t packet_number, uint8_t MAX_VEHICLES, Vehicle *vehicle_filters)
{
    uint8_t chosen =0;
    uint8_t found = 0;

    for (uint8_t i = 0 ; i < MAX_VEHICLES; i++)
    {
        if(vehicle_filters[i].used && vehicle_filters[i].vehicle_id == vehicle_id)
        {
            found = 1;
            chosen = i;
            break;
        }
    }
     
    if(!found){
        uint32_t oldest_packet = 0xFFFFFFFFUL;
        for (uint8_t i = 0; i < MAX_VEHICLES; i++)
        {
            if(!vehicle_filters[i].used)
            {
                chosen = i;
                break;
            }
            if(vehicle_filters[i].last_packet < oldest_packet)
            {
                oldest_packet = vehicle_filters[i].last_packet;
                chosen = i;
            }
        }
    
        vehicle_filters[chosen].used = 1;
        vehicle_filters[chosen].vehicle_id = vehicle_id;
        vehicle_filters[chosen].filter.sample_count = 0;
        vehicle_filters[chosen].filter.next_sample = 0;
        vehicle_filters[chosen].filter.estimate_value = 0;
        vehicle_filters[chosen].filter.covariance_value = 0;
        vehicle_filters[chosen].filter.kalman_initialized = 0;
    }

    vehicle_filters[chosen].last_packet = packet_number;
    return &vehicle_filters[chosen].filter;
}

static void remember_rssi(rssifilterstate *state, int16_t rssi)
{
    state->samples[state->next_sample] = rssi;
    state->next_sample = (state->next_sample +1) % 3; //wrap around the circular buffer of samples
    if(state->sample_count <3)
    {
        state->sample_count++;
    }
}

static int16_t get_average_rssi(rssifilterstate *state)
{
    int16_t values[3];
    
    for(uint8_t i = 0; i < state->sample_count; i++)
    {
        values[i] = state->samples[i];
    }

    for(uint8_t i = 0; i < state->sample_count; i++)
    {
        for(uint8_t j = i + 1; j < state->sample_count; j++)
        {
            if(values[j] < values[i])
            {
                int16_t temp = values[i];
                values[i] = values[j];
                values[j] = temp;
            }
        }
    }
    return values[state->sample_count / 2]; 
}

static int16_t moving_average_rssi(rssifilterstate *state)
{
    int32_t sum = 0;
    for (uint8_t i = 0; i < state->sample_count; i++)
    {
        sum += state->samples[i];
    }
    return (int16_t)(sum / state->sample_count);
}

static uint16_t snr_adaptive (int16_t snr, uint8_t lqi, uint8_t noise_valid)
{
    if (noise_valid && snr >= 3 && lqi >= 64)
    {
        if (snr >= 8 && lqi >= 128)
        {
            if(snr >=15 && lqi >= 192)
            {
                return 9; 
            }
            return 25;
        }
        return 49;
    } 
    return 100;
}

static uint8_t gaussian_likelihood(uint8_t residual_db)
{
    static const uint8_t likehood[] = {255, 248, 226, 199, 155, 120, 83, 54, 35, 21, 11, 6, 3, 1};

    if(residual_db >= sizeof(likehood))
    {
        return 1;
    }
    return likehood[residual_db];
}

static int16_t kalman_rssi(rssifilterstate *state, int16_t measurement_dbm, uint16_t measurement_variance, uint8_t gaussian_hybrid)
{
    if(!state -> kalman_initialized)
    {
        state->estimate_value = (int32_t)measurement_dbm * 256;
        state->covariance_value = (uint32_t)measurement_variance * 256;
        state->kalman_initialized = 1;
        return measurement_dbm;
    }
    if (gaussian_hybrid)
    {
        int32_t residual = (int32_t)measurement_dbm - state->estimate_value / 256;
        if (residual < 0)
        {
            residual = -residual;
        }
        uint8_t likelihood = gaussian_likelihood((uint8_t)residual );
        measurement_variance = (measurement_variance * 256UL) / likelihood;
        if (measurement_variance > 1000)
        {
            measurement_variance = 1000;
        }
    }

    uint32_t prior_covariance = state->covariance_value + 1024;
    uint32_t measurement_covariance = (uint32_t)measurement_variance * 256;

    uint32_t kalman_gain = (prior_covariance << 15) / (prior_covariance + measurement_covariance);
    int32_t residual = (int32_t)measurement_dbm * 256 - state->estimate_value;
    state->estimate_value += ((int32_t)kalman_gain * residual) >> 15;
    state->covariance_value = ((32768UL - kalman_gain) *prior_covariance) >> 15;

    return (int16_t)(state->estimate_value / 256);
    
}

static int16_t filter_rssi(rssifilterstate *state, int16_t raw_rssi, int16_t snr_db, uint8_t lqi, uint8_t noise_valid)
{
    remember_rssi(state, raw_rssi);

    switch(RSSI_FILTER_MODE){
        case RSSI_FILTER_RAW:
            return raw_rssi;
            break;
        case RSSI_FILTER_MEDIAN:
            return get_average_rssi(state);
            break;
        case RSSI_FILTER_MOVING_AVERAGE:
            return moving_average_rssi(state);
            break;
        case RSSI_FILTER_KALMAN:
            return kalman_rssi(state, raw_rssi, 9, 0);
            break;
        case RSSI_FILTER_HYBRID_GAUSSIAN_KALMAN:
            return kalman_rssi(state, raw_rssi, 9, 1);
            break; 
        case RSSI_FILTER_SNR_ADAPTIVE_KALMAN:
            return kalman_rssi(state, raw_rssi, snr_adaptive(snr_db, lqi, noise_valid), 0);
            break;
        default:
            return raw_rssi;
    }
}

#endif
