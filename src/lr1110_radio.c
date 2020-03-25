/*!
 * \file      lr1110_radio.c
 *
 * \brief     Radio driver implementation for LR1110
 *
 * Revised BSD License
 * Copyright Semtech Corporation 2020. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH S.A. BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr1110_radio.h"
#include "lr1110_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR1110_RADIO_RESET_STATS_CMD_LENGTH ( 2 )
#define LR1110_RADIO_GET_STATS_CMD_LENGTH ( 2 )
#define LR1110_RADIO_GET_PACKET_TYPE_CMD_LENGTH ( 2 )
#define LR1110_RADIO_GET_RXBUFFER_STATUS_CMD_LENGTH ( 2 )
#define LR1110_RADIO_GET_PACKET_STATUS_CMD_LENGTH ( 2 )
#define LR1110_RADIO_GET_RSSI_INST_CMD_LENGTH ( 2 )
#define LR1110_RADIO_SET_GFSK_SYNC_WORD_CMD_LENGTH ( 2 + 8 )
#define LR1110_RADIO_SET_LORA_SYNC_WORD_CMD_LENGTH ( 2 + 1 )
#define LR1110_RADIO_SET_RX_CMD_LENGTH ( 2 + 3 )
#define LR1110_RADIO_SET_TX_CMD_LENGTH ( 2 + 3 )
#define LR1110_RADIO_SET_RF_FREQUENCY_CMD_LENGTH ( 2 + 4 )
#define LR1110_RADIO_SET_AUTO_TX_RX_CMD_LENGTH ( 2 + 7 )
#define LR1110_RADIO_SET_CAD_PARAMS_CMD_LENGTH ( 2 + 7 )
#define LR1110_RADIO_SET_PACKET_TYPE_CMD_LENGTH ( 2 + 1 )
#define LR1110_RADIO_SET_MODULATION_PARAMS_GFSK_CMD_LENGTH ( 2 + 10 )
#define LR1110_RADIO_SET_MODULATION_PARAMS_LORA_CMD_LENGTH ( 2 + 4 )
#define LR1110_RADIO_SET_PACKET_PARAM_GFSK_CMD_LENGTH ( 2 + 9 )
#define LR1110_RADIO_SET_PACKET_PARAM_LORA_CMD_LENGTH ( 2 + 6 )
#define LR1110_RADIO_SET_TX_PARAMS_CMD_LENGTH ( 2 + 2 )
#define LR1110_RADIO_SET_PACKET_ADDRESS_CMD_LENGTH ( 2 + 2 )
#define LR1110_RADIO_SET_RX_TX_FALLBACK_MODE_CMD_LENGTH ( 2 + 1 )
#define LR1110_RADIO_SET_RX_DUTYCYCLE_MODE_CMD_LENGTH ( 2 + 7 )
#define LR1110_RADIO_SET_PA_CONFIG_CMD_LENGTH ( 2 + 4 )
#define LR1110_RADIO_STOP_TIMEOUT_ON_PREAMBLE_CMD_LENGTH ( 2 + 1 )
#define LR1110_RADIO_SET_CAD_CMD_LENGTH ( 2 )
#define LR1110_RADIO_SET_TX_CW_CMD_LENGTH ( 2 )
#define LR1110_RADIO_SET_TX_INFINITE_PREAMBLE_CMD_LENGTH ( 2 )
#define LR1110_RADIO_SET_LORA_SYNC_TIMEOUT_CMD_LENGTH ( 2 + 1 )
#define LR1110_RADIO_SET_GFSK_CRC_PARAMS_CMD_LENGTH ( 2 + 8 )
#define LR1110_RADIO_SET_GFSK_WHITENING_CMD_LENGTH ( 2 + 2 )
#define LR1110_RADIO_SET_RX_BOOSTED_LENGTH ( 2 + 1 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

enum
{
    LR1110_RADIO_RESET_STATS_OC               = 0x0200,
    LR1110_RADIO_GET_STATS_OC                 = 0x0201,
    LR1110_RADIO_GET_PACKETTYPE_OC            = 0x0202,
    LR1110_RADIO_GET_RXBUFFER_STATUS_OC       = 0x0203,
    LR1110_RADIO_GET_PACKET_STATUS_OC         = 0x0204,
    LR1110_RADIO_GET_RSSI_INST_OC             = 0x0205,
    LR1110_RADIO_SET_GFSK_SYNC_WORD_OC        = 0x0206,
    LR1110_RADIO_SET_LORA_SYNC_WORD_OC        = 0x0208,
    LR1110_RADIO_SET_RX_OC                    = 0x0209,
    LR1110_RADIO_SET_TX_OC                    = 0x020A,
    LR1110_RADIO_SET_RF_FREQUENCY_OC          = 0x020B,
    LR1110_RADIO_AUTOTXRX_OC                  = 0x020C,
    LR1110_RADIO_SET_CAD_PARAMS_OC            = 0x020D,
    LR1110_RADIO_SET_PACKET_TYPE_OC           = 0x020E,
    LR1110_RADIO_SET_MODULATION_PARAM_OC      = 0x020F,
    LR1110_RADIO_SET_PACKET_PARAM_OC          = 0x0210,
    LR1110_RADIO_SET_TX_PARAMS_OC             = 0x0211,
    LR1110_RADIO_SET_PACKETADRS_OC            = 0x0212,
    LR1110_RADIO_SET_RX_TX_FALLBACK_MODE_OC   = 0x0213,
    LR1110_RADIO_SET_RX_DUTYCYCLE_OC          = 0x0214,
    LR1110_RADIO_SET_PACONFIG_OC              = 0x0215,
    LR1110_RADIO_STOP_TIMEOUT_ON_PREAMBLE_OC  = 0x0217,
    LR1110_RADIO_SET_CAD_OC                   = 0x0218,
    LR1110_RADIO_SET_TX_CW_OC                 = 0x0219,
    LR1110_RADIO_SET_TX_INFINITE_PREAMBLE_OC  = 0x021A,
    LR1110_RADIO_SET_LORA_SYNC_TIMEOUT_OC     = 0x021B,
    LR1110_RADIO_SET_GFSK_CRC_PARAMS_OC       = 0x0224,
    LR1110_RADIO_SET_GFSK_WHITENING_PARAMS_OC = 0x0225,
    LR1110_RADIO_SET_RX_BOOSTED_OC            = 0x0227,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void lr1110_radio_reset_stats( const void* radio )
{
    uint8_t cbuffer[LR1110_RADIO_RESET_STATS_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_RESET_STATS_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_RESET_STATS_OC >> 0 );

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_RESET_STATS_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_get_stats_gfsk( const void* radio, lr1110_radio_stats_gfsk_t* radio_stats )
{
    uint8_t cbuffer[LR1110_RADIO_GET_STATS_CMD_LENGTH];
    uint8_t rbuffer[sizeof( lr1110_radio_stats_gfsk_t )] = { 0x00 };

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_GET_STATS_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_GET_STATS_OC >> 0 );

    lr1110_hal_read( radio, cbuffer, LR1110_RADIO_GET_STATS_CMD_LENGTH, rbuffer, sizeof( lr1110_radio_stats_gfsk_t ) );

    radio_stats->nb_packet_received     = ( ( uint16_t ) rbuffer[0] << 8 ) + ( uint16_t ) rbuffer[1];
    radio_stats->nb_packet_error_crc    = ( ( uint16_t ) rbuffer[2] << 8 ) + ( uint16_t ) rbuffer[3];
    radio_stats->nb_packet_error_length = ( ( uint16_t ) rbuffer[4] << 8 ) + ( uint16_t ) rbuffer[5];
}

void lr1110_radio_get_stats_lora( const void* radio, lr1110_radio_stats_lora_t* radio_stats )
{
    uint8_t cbuffer[LR1110_RADIO_GET_STATS_CMD_LENGTH];
    uint8_t rbuffer[sizeof( lr1110_radio_stats_lora_t )] = { 0x00 };

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_GET_STATS_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_GET_STATS_OC >> 0 );

    lr1110_hal_read( radio, cbuffer, LR1110_RADIO_GET_STATS_CMD_LENGTH, rbuffer, sizeof( lr1110_radio_stats_lora_t ) );

    radio_stats->nb_packet_received     = ( ( uint16_t ) rbuffer[0] << 8 ) + ( uint16_t ) rbuffer[1];
    radio_stats->nb_packet_error_crc    = ( ( uint16_t ) rbuffer[2] << 8 ) + ( uint16_t ) rbuffer[3];
    radio_stats->nb_packet_error_header = ( ( uint16_t ) rbuffer[4] << 8 ) + ( uint16_t ) rbuffer[5];
    radio_stats->nb_packet_falsesync    = ( ( uint16_t ) rbuffer[6] << 8 ) + ( uint16_t ) rbuffer[7];
}

void lr1110_radio_get_packet_type( const void* radio, lr1110_radio_packet_types_t* packet_type )
{
    uint8_t cbuffer[LR1110_RADIO_GET_PACKET_TYPE_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_GET_PACKETTYPE_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_GET_PACKETTYPE_OC >> 0 );

    lr1110_hal_read( radio, cbuffer, LR1110_RADIO_GET_PACKET_TYPE_CMD_LENGTH, ( uint8_t* ) packet_type,
                     sizeof( uint8_t ) );
}

void lr1110_radio_get_rxbuffer_status( const void* radio, lr1110_radio_rxbuffer_status_t* rxbuffer_status )
{
    uint8_t cbuffer[LR1110_RADIO_GET_RXBUFFER_STATUS_CMD_LENGTH];
    uint8_t rbuffer[sizeof( *rxbuffer_status )] = { 0x00 };

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_GET_RXBUFFER_STATUS_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_GET_RXBUFFER_STATUS_OC >> 0 );

    lr1110_hal_read( radio, cbuffer, LR1110_RADIO_GET_RXBUFFER_STATUS_CMD_LENGTH, rbuffer, sizeof( *rxbuffer_status ) );

    rxbuffer_status->rx_payload_length       = rbuffer[0];
    rxbuffer_status->rx_start_buffer_pointer = rbuffer[1];
}

void lr1110_radio_get_packet_status_gfsk( const void* radio, lr1110_radio_packet_status_gfsk_t* packet_status )
{
    uint8_t cbuffer[LR1110_RADIO_GET_PACKET_STATUS_CMD_LENGTH];
    uint8_t rbuffer[4] = { 0x00 };

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_GET_PACKET_STATUS_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_GET_PACKET_STATUS_OC >> 0 );

    lr1110_hal_read( radio, cbuffer, LR1110_RADIO_GET_PACKET_STATUS_CMD_LENGTH, rbuffer, 4 );

    packet_status->rssi_sync_in_dbm   = -( int8_t )( rbuffer[0] >> 1 );
    packet_status->rssi_avg_in_dbm    = -( int8_t )( rbuffer[1] >> 1 );
    packet_status->rx_length_in_bytes = rbuffer[2];
    packet_status->is_addr_err        = ( ( rbuffer[3] & 0x20 ) != 0 ) ? true : false;
    packet_status->is_crc_err         = ( ( rbuffer[3] & 0x10 ) != 0 ) ? true : false;
    packet_status->is_len_err         = ( ( rbuffer[3] & 0x08 ) != 0 ) ? true : false;
    packet_status->is_abort_err       = ( ( rbuffer[3] & 0x04 ) != 0 ) ? true : false;
    packet_status->is_received        = ( ( rbuffer[3] & 0x02 ) != 0 ) ? true : false;
    packet_status->is_sent            = ( ( rbuffer[3] & 0x01 ) != 0 ) ? true : false;
}

void lr1110_radio_get_packet_status_lora( const void* radio, lr1110_radio_packet_status_lora_t* packet_status )
{
    uint8_t cbuffer[LR1110_RADIO_GET_PACKET_STATUS_CMD_LENGTH];
    uint8_t rbuffer[3] = { 0x00 };

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_GET_PACKET_STATUS_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_GET_PACKET_STATUS_OC >> 0 );

    lr1110_hal_read( radio, cbuffer, LR1110_RADIO_GET_PACKET_STATUS_CMD_LENGTH, rbuffer, 3 );

    packet_status->rssi_packet_in_dbm        = -( int8_t )( rbuffer[0] >> 1 );
    packet_status->snr_packet_in_db          = ( ( ( int8_t ) rbuffer[1] ) + 2 ) >> 2;
    packet_status->signal_rssi_packet_in_dbm = -( int8_t )( rbuffer[2] >> 1 );
}

void lr1110_radio_get_rssi_inst( const void* radio, int8_t* rssi_in_dbm )
{
    uint8_t cbuffer[LR1110_RADIO_GET_RSSI_INST_CMD_LENGTH];
    uint8_t rssi = 0;

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_GET_RSSI_INST_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_GET_RSSI_INST_OC >> 0 );

    lr1110_hal_read( radio, cbuffer, LR1110_RADIO_GET_RSSI_INST_CMD_LENGTH, &rssi, sizeof( rssi ) );
    *rssi_in_dbm = -( int8_t )( rssi >> 1 );
}

void lr1110_radio_set_gfsk_sync_word( const void* radio, const uint8_t* gfsk_sync_word )
{
    uint8_t cbuffer[LR1110_RADIO_SET_GFSK_SYNC_WORD_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_SET_GFSK_SYNC_WORD_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_SET_GFSK_SYNC_WORD_OC >> 0 );

    for( uint8_t index = 0; index < 8; index++ )
    {
        cbuffer[2 + index] = gfsk_sync_word[index];
    }

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_GFSK_SYNC_WORD_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_set_lora_sync_word( const void* radio, const lr1110_radio_lora_network_type_t network_type )
{
    uint8_t cbuffer[LR1110_RADIO_SET_LORA_SYNC_WORD_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_SET_LORA_SYNC_WORD_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_SET_LORA_SYNC_WORD_OC >> 0 );

    cbuffer[2] = ( uint8_t ) network_type;

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_LORA_SYNC_WORD_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_set_rx( const void* radio, const uint32_t timeout )
{
    uint8_t cbuffer[LR1110_RADIO_SET_RX_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_SET_RX_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_SET_RX_OC >> 0 );

    cbuffer[2] = ( uint8_t )( timeout >> 16 );
    cbuffer[3] = ( uint8_t )( timeout >> 8 );
    cbuffer[4] = ( uint8_t )( timeout >> 0 );

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_RX_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_set_tx( const void* radio, const uint32_t timeout )
{
    uint8_t cbuffer[LR1110_RADIO_SET_TX_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_SET_TX_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_SET_TX_OC >> 0 );

    cbuffer[2] = ( uint8_t )( timeout >> 16 );
    cbuffer[3] = ( uint8_t )( timeout >> 8 );
    cbuffer[4] = ( uint8_t )( timeout >> 0 );

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_TX_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_set_rf_frequency( const void* radio, const uint32_t frequency_in_hz )
{
    uint8_t cbuffer[LR1110_RADIO_SET_RF_FREQUENCY_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_SET_RF_FREQUENCY_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_SET_RF_FREQUENCY_OC >> 0 );

    cbuffer[2] = ( uint8_t )( frequency_in_hz >> 24 );
    cbuffer[3] = ( uint8_t )( frequency_in_hz >> 16 );
    cbuffer[4] = ( uint8_t )( frequency_in_hz >> 8 );
    cbuffer[5] = ( uint8_t )( frequency_in_hz >> 0 );

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_RF_FREQUENCY_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_auto_tx_rx( const void* radio, const uint32_t delay,
                              const lr1110_radio_intermediary_mode_t intermediary_mode, const uint32_t timeout )
{
    uint8_t cbuffer[LR1110_RADIO_SET_AUTO_TX_RX_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_AUTOTXRX_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_AUTOTXRX_OC >> 0 );

    cbuffer[2] = ( uint8_t )( delay >> 16 );
    cbuffer[3] = ( uint8_t )( delay >> 8 );
    cbuffer[4] = ( uint8_t )( delay );

    cbuffer[5] = ( uint8_t ) intermediary_mode;

    cbuffer[6] = ( uint8_t )( timeout >> 16 );
    cbuffer[7] = ( uint8_t )( timeout >> 8 );
    cbuffer[8] = ( uint8_t )( timeout );

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_AUTO_TX_RX_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_set_cad_params( const void* radio, const lr1110_radio_cad_params_t* cad_params )
{
    uint8_t cbuffer[LR1110_RADIO_SET_CAD_PARAMS_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_SET_CAD_PARAMS_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_SET_CAD_PARAMS_OC >> 0 );

    cbuffer[2] = cad_params->symbol_num;
    cbuffer[3] = cad_params->det_peak;
    cbuffer[4] = cad_params->det_min;
    cbuffer[5] = cad_params->exit_mode;

    cbuffer[6] = ( uint8_t )( cad_params->timeout >> 16 );
    cbuffer[7] = ( uint8_t )( cad_params->timeout >> 8 );
    cbuffer[8] = ( uint8_t )( cad_params->timeout );

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_CAD_PARAMS_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_set_packet_type( const void* radio, const lr1110_radio_packet_types_t packet_type )
{
    uint8_t cbuffer[LR1110_RADIO_SET_PACKET_TYPE_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_SET_PACKET_TYPE_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_SET_PACKET_TYPE_OC >> 0 );

    cbuffer[2] = ( uint8_t ) packet_type;

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_PACKET_TYPE_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_set_modulation_param_gfsk( const void*                                 radio,
                                             const lr1110_radio_modulation_param_gfsk_t* modulation_params )
{
    uint8_t cbuffer[LR1110_RADIO_SET_MODULATION_PARAMS_GFSK_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_SET_MODULATION_PARAM_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_SET_MODULATION_PARAM_OC >> 0 );

    cbuffer[2] = ( uint8_t )( modulation_params->bitrate >> 24 );
    cbuffer[3] = ( uint8_t )( modulation_params->bitrate >> 16 );
    cbuffer[4] = ( uint8_t )( modulation_params->bitrate >> 8 );
    cbuffer[5] = ( uint8_t )( modulation_params->bitrate >> 0 );

    cbuffer[6] = ( uint8_t ) modulation_params->pulse_shape;
    cbuffer[7] = ( uint8_t ) modulation_params->bandwidth;

    cbuffer[8]  = ( uint8_t )( modulation_params->fdev >> 24 );
    cbuffer[9]  = ( uint8_t )( modulation_params->fdev >> 16 );
    cbuffer[10] = ( uint8_t )( modulation_params->fdev >> 8 );
    cbuffer[11] = ( uint8_t )( modulation_params->fdev >> 0 );

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_MODULATION_PARAMS_GFSK_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_set_modulation_param_lora( const void*                                 radio,
                                             const lr1110_radio_modulation_param_lora_t* modulation_params )
{
    uint8_t cbuffer[LR1110_RADIO_SET_MODULATION_PARAMS_LORA_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_SET_MODULATION_PARAM_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_SET_MODULATION_PARAM_OC >> 0 );

    cbuffer[2] = ( uint8_t ) modulation_params->spreading_factor;
    cbuffer[3] = ( uint8_t ) modulation_params->bandwidth;
    cbuffer[4] = ( uint8_t ) modulation_params->coding_rate;
    cbuffer[5] = ( uint8_t ) modulation_params->ppm_offset;

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_MODULATION_PARAMS_LORA_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_set_packet_param_gfsk( const void* radio, const lr1110_radio_packet_param_gfsk_t* packet_params )
{
    uint8_t cbuffer[LR1110_RADIO_SET_PACKET_PARAM_GFSK_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_SET_PACKET_PARAM_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_SET_PACKET_PARAM_OC >> 0 );

    cbuffer[2] = ( uint8_t )( packet_params->preamble_length_tx_in_bit >> 8 );
    cbuffer[3] = ( uint8_t )( packet_params->preamble_length_tx_in_bit >> 0 );

    cbuffer[4] = ( uint8_t )( packet_params->preamble_detect );

    cbuffer[5] = packet_params->sync_word_length_in_bit;

    cbuffer[6] = ( uint8_t )( packet_params->address_filtering );

    cbuffer[7] = ( uint8_t )( packet_params->header_type );

    cbuffer[8] = packet_params->payload_length_in_byte;

    cbuffer[9] = ( uint8_t )( packet_params->crc_type );

    cbuffer[10] = ( uint8_t )( packet_params->dc_free );

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_PACKET_PARAM_GFSK_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_set_packet_param_lora( const void* radio, const lr1110_radio_packet_param_lora_t* packet_params )
{
    uint8_t cbuffer[LR1110_RADIO_SET_PACKET_PARAM_LORA_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_SET_PACKET_PARAM_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_SET_PACKET_PARAM_OC >> 0 );

    cbuffer[2] = ( uint8_t )( packet_params->preamble_length_in_symb >> 8 );
    cbuffer[3] = ( uint8_t )( packet_params->preamble_length_in_symb >> 0 );

    cbuffer[4] = ( uint8_t )( packet_params->header_type );

    cbuffer[5] = packet_params->payload_length_in_byte;

    cbuffer[6] = ( uint8_t )( packet_params->crc );

    cbuffer[7] = ( uint8_t )( packet_params->iq );

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_PACKET_PARAM_LORA_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_set_tx_params( const void* radio, const int8_t power, const lr1110_radio_ramp_time_t ramp_time )
{
    uint8_t cbuffer[LR1110_RADIO_SET_TX_PARAMS_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_SET_TX_PARAMS_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_SET_TX_PARAMS_OC >> 0 );

    cbuffer[2] = ( uint8_t ) power;
    cbuffer[3] = ( uint8_t ) ramp_time;

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_TX_PARAMS_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_set_packet_address( const void* radio, const uint8_t node_address, const uint8_t broadcast_address )
{
    uint8_t cbuffer[LR1110_RADIO_SET_PACKET_ADDRESS_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_SET_PACKETADRS_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_SET_PACKETADRS_OC >> 0 );

    cbuffer[2] = node_address;
    cbuffer[3] = broadcast_address;

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_PACKET_ADDRESS_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_set_rx_tx_fallback_mode( const void* radio, const lr1110_radio_rx_tx_fallback_mode_t fallback_mode )
{
    uint8_t cbuffer[LR1110_RADIO_SET_RX_TX_FALLBACK_MODE_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_SET_RX_TX_FALLBACK_MODE_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_SET_RX_TX_FALLBACK_MODE_OC >> 0 );

    cbuffer[2] = fallback_mode;

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_RX_TX_FALLBACK_MODE_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_set_rx_dutycycle( const void* radio, const uint32_t rx_period, const uint32_t sleep_period,
                                    const lr1110_radio_rx_duty_cycle_mode_t mode )
{
    uint8_t cbuffer[LR1110_RADIO_SET_RX_DUTYCYCLE_MODE_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_SET_RX_DUTYCYCLE_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_SET_RX_DUTYCYCLE_OC >> 0 );

    cbuffer[2] = ( uint8_t )( rx_period >> 16 );
    cbuffer[3] = ( uint8_t )( rx_period >> 8 );
    cbuffer[4] = ( uint8_t )( rx_period >> 0 );

    cbuffer[5] = ( uint8_t )( sleep_period >> 16 );
    cbuffer[6] = ( uint8_t )( sleep_period >> 8 );
    cbuffer[7] = ( uint8_t )( sleep_period >> 0 );

    cbuffer[8] = mode;

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_RX_DUTYCYCLE_MODE_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_set_pa_config( const void* radio, const lr1110_radio_pa_config_t* pa_config )
{
    uint8_t cbuffer[LR1110_RADIO_SET_PA_CONFIG_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_SET_PACONFIG_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_SET_PACONFIG_OC >> 0 );

    cbuffer[2] = ( uint8_t ) pa_config->pa_sel;
    cbuffer[3] = ( uint8_t ) pa_config->pa_reg_supply;

    cbuffer[4] = pa_config->pa_dutycycle;
    cbuffer[5] = pa_config->pa_hp_sel;

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_PA_CONFIG_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_stop_timeout_on_preamble( const void* radio, const bool stop_timeout_on_preamble )
{
    uint8_t cbuffer[LR1110_RADIO_STOP_TIMEOUT_ON_PREAMBLE_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_STOP_TIMEOUT_ON_PREAMBLE_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_STOP_TIMEOUT_ON_PREAMBLE_OC >> 0 );

    cbuffer[2] = ( uint8_t ) stop_timeout_on_preamble;

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_STOP_TIMEOUT_ON_PREAMBLE_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_set_cad( const void* radio )
{
    uint8_t cbuffer[LR1110_RADIO_SET_CAD_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_SET_CAD_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_SET_CAD_OC >> 0 );

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_CAD_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_set_tx_cw( const void* radio )
{
    uint8_t cbuffer[LR1110_RADIO_SET_TX_CW_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_SET_TX_CW_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_SET_TX_CW_OC >> 0 );

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_TX_CW_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_set_tx_infinite_preamble( const void* radio )
{
    uint8_t cbuffer[LR1110_RADIO_SET_TX_INFINITE_PREAMBLE_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_SET_TX_INFINITE_PREAMBLE_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_SET_TX_INFINITE_PREAMBLE_OC >> 0 );

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_TX_INFINITE_PREAMBLE_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_set_lora_sync_timeout( const void* radio, const uint8_t nb_symbol )
{
    uint8_t cbuffer[LR1110_RADIO_SET_LORA_SYNC_TIMEOUT_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_SET_LORA_SYNC_TIMEOUT_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_SET_LORA_SYNC_TIMEOUT_OC >> 0 );

    cbuffer[2] = nb_symbol;

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_LORA_SYNC_TIMEOUT_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_set_gfsk_crc_params( const void* radio, const uint32_t seed, const uint32_t polynomial )
{
    uint8_t cbuffer[LR1110_RADIO_SET_GFSK_CRC_PARAMS_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_SET_GFSK_CRC_PARAMS_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_SET_GFSK_CRC_PARAMS_OC >> 0 );

    cbuffer[2] = ( uint8_t )( seed >> 24 );
    cbuffer[3] = ( uint8_t )( seed >> 16 );
    cbuffer[4] = ( uint8_t )( seed >> 8 );
    cbuffer[5] = ( uint8_t )( seed >> 0 );

    cbuffer[6] = ( uint8_t )( polynomial >> 24 );
    cbuffer[7] = ( uint8_t )( polynomial >> 16 );
    cbuffer[8] = ( uint8_t )( polynomial >> 8 );
    cbuffer[9] = ( uint8_t )( polynomial >> 0 );

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_GFSK_CRC_PARAMS_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_set_gfsk_whitening_params( const void* radio, const uint16_t whitening )
{
    uint8_t cbuffer[LR1110_RADIO_SET_GFSK_WHITENING_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_SET_GFSK_WHITENING_PARAMS_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_SET_GFSK_WHITENING_PARAMS_OC >> 0 );

    cbuffer[2] = ( uint8_t )( whitening >> 8 );
    cbuffer[3] = ( uint8_t )( whitening >> 0 );

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_GFSK_WHITENING_CMD_LENGTH, 0, 0 );
}

void lr1110_radio_set_rx_boosted( const void* radio, const bool enable_boost_mode )
{
    uint8_t cbuffer[LR1110_RADIO_SET_RX_BOOSTED_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_RADIO_SET_RX_BOOSTED_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_RADIO_SET_RX_BOOSTED_OC >> 0 );

    cbuffer[2] = ( enable_boost_mode == true ) ? 0x01 : 0x00;

    lr1110_hal_write( radio, cbuffer, LR1110_RADIO_SET_RX_BOOSTED_LENGTH, 0, 0 );
}

void lr1110_radio_get_gfsk_rx_bandwidth( uint32_t bw_in_hz, lr1110_radio_gfsk_rx_bw_t* bw_parameter )
{
    if( bw_in_hz <= 4800 )
    {
        *bw_parameter = LR1110_RADIO_GFSK_RX_BW_4800;
    }
    else if( bw_in_hz <= 5800 )
    {
        *bw_parameter = LR1110_RADIO_GFSK_RX_BW_5800;
    }
    else if( bw_in_hz <= 7300 )
    {
        *bw_parameter = LR1110_RADIO_GFSK_RX_BW_7300;
    }
    else if( bw_in_hz <= 9700 )
    {
        *bw_parameter = LR1110_RADIO_GFSK_RX_BW_9700;
    }
    else if( bw_in_hz <= 11700 )
    {
        *bw_parameter = LR1110_RADIO_GFSK_RX_BW_11700;
    }
    else if( bw_in_hz <= 14600 )
    {
        *bw_parameter = LR1110_RADIO_GFSK_RX_BW_14600;
    }
    else if( bw_in_hz <= 19500 )
    {
        *bw_parameter = LR1110_RADIO_GFSK_RX_BW_19500;
    }
    else if( bw_in_hz <= 23400 )
    {
        *bw_parameter = LR1110_RADIO_GFSK_RX_BW_23400;
    }
    else if( bw_in_hz <= 29300 )
    {
        *bw_parameter = LR1110_RADIO_GFSK_RX_BW_29300;
    }
    else if( bw_in_hz <= 39000 )
    {
        *bw_parameter = LR1110_RADIO_GFSK_RX_BW_39000;
    }
    else if( bw_in_hz <= 46900 )
    {
        *bw_parameter = LR1110_RADIO_GFSK_RX_BW_46900;
    }
    else if( bw_in_hz <= 58600 )
    {
        *bw_parameter = LR1110_RADIO_GFSK_RX_BW_58600;
    }
    else if( bw_in_hz <= 78200 )
    {
        *bw_parameter = LR1110_RADIO_GFSK_RX_BW_78200;
    }
    else if( bw_in_hz <= 93800 )
    {
        *bw_parameter = LR1110_RADIO_GFSK_RX_BW_93800;
    }
    else if( bw_in_hz <= 117300 )
    {
        *bw_parameter = LR1110_RADIO_GFSK_RX_BW_117300;
    }
    else if( bw_in_hz <= 156200 )
    {
        *bw_parameter = LR1110_RADIO_GFSK_RX_BW_156200;
    }
    else if( bw_in_hz <= 187200 )
    {
        *bw_parameter = LR1110_RADIO_GFSK_RX_BW_187200;
    }
    else if( bw_in_hz <= 234300 )
    {
        *bw_parameter = LR1110_RADIO_GFSK_RX_BW_234300;
    }
    else if( bw_in_hz <= 312000 )
    {
        *bw_parameter = LR1110_RADIO_GFSK_RX_BW_312000;
    }
    else if( bw_in_hz <= 373600 )
    {
        *bw_parameter = LR1110_RADIO_GFSK_RX_BW_373600;
    }
    else if( bw_in_hz <= 467000 )
    {
        *bw_parameter = LR1110_RADIO_GFSK_RX_BW_467000;
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
