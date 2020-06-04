/*!
 * \file      lr1110_wifi.c
 *
 * \brief     Wi-Fi passive scan driver implementation for LR1110
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

#include "lr1110_wifi.h"
#include "lr1110_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#ifndef MIN
#define MIN( a, b ) ( ( a > b ) ? b : a )
#endif  // MIN

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR1110_WIFI_BASIC_COMPLETE_RESULT_SIZE ( 22 )
#define LR1110_WIFI_BASIC_MAC_TYPE_CHANNEL_RESULT_SIZE ( 9 )

#define LR1110_WIFI_MAX_SIZE_PER_SPI( single_size ) \
    ( single_size * ( LR1110_WIFI_MAX_RESULT_PER_TRANSACTION( single_size ) ) )

#define LR1110_WIFI_MAX_RESULT_PER_TRANSACTION( single_size ) \
    ( MIN( ( LR1110_WIFI_READ_RESULT_LIMIT ) / ( single_size ), LR1110_WIFI_N_RESULTS_MAX_PER_CHUNK ) )

#define LR1110_WIFI_ALL_CUMULATIVE_TIMING_SIZE ( 16 )
#define LR1110_WIFI_VERSION_SIZE ( 2 )
#define LR1110_WIFI_READ_RESULT_LIMIT ( 1020 )
#define LR1110_WIFI_COUNTRY_RESULT_LENGTH_SIZE ( 1 )
#define LR1110_WIFI_SCAN_SINGLE_COUNTRY_CODE_RESULT_SIZE ( 10 )
#define LR1110_WIFI_MAX_COUNTRY_CODE_RESULT_SIZE \
    ( LR1110_WIFI_MAX_COUNTRY_CODE * LR1110_WIFI_SCAN_SINGLE_COUNTRY_CODE_RESULT_SIZE )

// Command length
#define LR1110_WIFI_SCAN_CMD_LENGTH ( 2 + 9 )
#define LR1110_WIFI_SEARCH_COUNTRY_CODE_CMD_LENGTH ( 2 + 7 )
#define LR1110_WIFI_CONFIGURE_HARDWARE_DEBARKER_CMD_LENGTH ( 2 + 1 )
#define LR1110_WIFI_GET_RESULT_SIZE_CMD_LENGTH ( 2 )
#define LR1110_WIFI_READ_RESULT_CMD_LENGTH ( 2 + 3 )
#define LR1110_WIFI_RESET_CUMUL_TIMING_CMD_LENGTH ( 2 )
#define LR1110_WIFI_READ_CUMUL_TIMING_CMD_LENGTH ( 2 )
#define LR1110_WIFI_GET_SIZE_COUNTRY_RESULT_CMD_LENGTH ( 2 )
#define LR1110_WIFI_READ_COUNTRY_CODE_CMD_LENGTH ( 2 + 2 )
#define LR1110_WIFI_CONFIG_TIMESTAMP_AP_PHONE_CMD_LENGTH ( 2 + 4 )
#define LR1110_WIFI_GET_VERSION_CMD_LENGTH ( 2 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef enum
{
    LR1110_WIFI_SCAN_OC                         = 0x0300,
    LR1110_WIFI_SEARCH_COUNTRY_CODE_OC          = 0x0302,
    LR1110_WIFI_CONFIGURE_HARDWARE_DEBARKER_OC  = 0x0304,
    LR1110_WIFI_GET_RESULT_SIZE_OC              = 0x0305,
    LR1110_WIFI_READ_RESULT_OC                  = 0x0306,
    LR1110_WIFI_RESET_CUMUL_TIMING_OC           = 0x0307,
    LR1110_WIFI_READ_CUMUL_TIMING_OC            = 0x0308,
    LR1110_WIFI_GET_SIZE_COUNTRY_RESULT_OC      = 0x0309,
    LR1110_WIFI_READ_COUNTRY_CODE_OC            = 0x030A,
    LR1110_WIFI_CONFIGURE_TIMESTAMP_AP_PHONE_OC = 0x030B,
    LR1110_WIFI_GET_VERSION_OC                  = 0x0320,
} WifiOpcodes_t;

typedef union
{
    lr1110_wifi_basic_complete_result_t*         basic_complete;
    lr1110_wifi_basic_mac_type_channel_result_t* basic_mac_type_channel;
} lr1110_wifi_result_interface_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * \brief Return a uint16 value by reading a buffer of uint8 from index.
 *
 * This function interpret the array MSB first. It is equivalent to:
 * return array[index] * 256 + array[index+1]
 *
 */
static uint16_t uint16_from_array( const uint8_t* array, const uint16_t index );

/*!
 * \brief Return a uint64 value by reading a buffer of uint8 from index.
 *
 * This function interpret the array MSB first.
 *
 */
static uint64_t uint64_from_array( const uint8_t* array, const uint16_t index );

static void generic_results_interpreter( const uint8_t n_result_to_parse, const uint8_t* buffer,
                                         lr1110_wifi_result_interface_t    result_interface,
                                         const lr1110_wifi_result_format_t format_code );

static void interpret_basic_complete_result_from_buffer( const uint8_t nb_results, const uint8_t* buffer,
                                                         lr1110_wifi_basic_complete_result_t* result );

static void interpret_basic_mac_type_channel_result_from_buffer( const uint8_t nb_results, const uint8_t* buffer,
                                                                 lr1110_wifi_basic_mac_type_channel_result_t* result );

static lr1110_status_t fetch_and_aggregate_all_results( const void* context, const uint8_t index_result_start,
                                                        const uint8_t                     nb_results,
                                                        const uint8_t                     nb_results_per_chunk_max,
                                                        const lr1110_wifi_result_format_t result_format_code,
                                                        uint8_t*                          result_buffer,
                                                        lr1110_wifi_result_interface_t    result_structures );

static uint8_t lr1110_wifi_get_result_size_from_format( const lr1110_wifi_result_format_t format );

static lr1110_hal_status_t lr1110_wifi_read_results_helper( const void* context, const uint8_t start_index,
                                                            const uint8_t n_elem, uint8_t* buffer,
                                                            const lr1110_wifi_result_format_t result_format );

static void lr1110_wifi_read_mac_address_from_buffer( const uint8_t* buffer, const uint16_t index_in_buffer,
                                                      lr1110_wifi_mac_address_t mac_address );

static uint8_t lr1110_wifi_get_format_code( const lr1110_wifi_result_format_t format );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr1110_status_t lr1110_wifi_scan( const void* context, const lr1110_wifi_signal_type_scan_t signal_type,
                                  const lr1110_wifi_channel_mask_t channels, const lr1110_wifi_mode_t scan_mode,
                                  const uint8_t max_results, const uint8_t nb_scan_per_channel,
                                  const uint16_t timeout_in_ms, const bool abort_on_timeout )
{
    const uint8_t cbuffer[LR1110_WIFI_SCAN_CMD_LENGTH] = {
        ( uint8_t )( LR1110_WIFI_SCAN_OC >> 8 ),
        ( uint8_t ) LR1110_WIFI_SCAN_OC,
        ( uint8_t ) signal_type,
        ( uint8_t )( channels >> 8 ),
        ( uint8_t ) channels,
        ( uint8_t ) scan_mode,
        ( uint8_t ) max_results,
        ( uint8_t ) nb_scan_per_channel,
        ( uint8_t )( timeout_in_ms >> 8 ),
        ( uint8_t ) timeout_in_ms,
        ( uint8_t ) abort_on_timeout,
    };
    return ( lr1110_status_t ) lr1110_hal_write( context, cbuffer, LR1110_WIFI_SCAN_CMD_LENGTH, 0, 0 );
}

lr1110_status_t lr1110_wifi_search_country_code( const void* context, const lr1110_wifi_channel_mask_t channels_mask,
                                                 const uint8_t nb_max_results, const uint8_t nb_scan_per_channel,
                                                 const uint16_t timeout_in_ms, const bool abort_on_timeout )
{
    const uint8_t cbuffer[LR1110_WIFI_SEARCH_COUNTRY_CODE_CMD_LENGTH] = {
        ( uint8_t )( LR1110_WIFI_SEARCH_COUNTRY_CODE_OC >> 8 ),
        ( uint8_t ) LR1110_WIFI_SEARCH_COUNTRY_CODE_OC,
        ( uint8_t )( channels_mask >> 8 ),
        ( uint8_t ) channels_mask,
        nb_max_results,
        nb_scan_per_channel,
        ( uint8_t )( timeout_in_ms >> 8 ),
        ( uint8_t ) timeout_in_ms,
        ( uint8_t )( abort_on_timeout ? 1 : 0 )
    };
    return ( lr1110_status_t ) lr1110_hal_write( context, cbuffer, LR1110_WIFI_SEARCH_COUNTRY_CODE_CMD_LENGTH, 0, 0 );
}

lr1110_status_t lr1110_wifi_cfg_hardware_debarker( const void* context, const bool enable_hardware_debarker )
{
    const uint8_t cbuffer[LR1110_WIFI_CONFIGURE_HARDWARE_DEBARKER_CMD_LENGTH] = {
        ( uint8_t )( LR1110_WIFI_CONFIGURE_HARDWARE_DEBARKER_OC >> 8 ),
        ( uint8_t )( LR1110_WIFI_CONFIGURE_HARDWARE_DEBARKER_OC & 0x00FF ),
        ( uint8_t )( enable_hardware_debarker ? 1 : 0 )
    };
    return ( lr1110_status_t ) lr1110_hal_write( context, cbuffer, LR1110_WIFI_CONFIGURE_HARDWARE_DEBARKER_CMD_LENGTH,
                                                 0, 0 );
}

lr1110_status_t lr1110_wifi_get_nb_results( const void* context, uint8_t* nb_results )
{
    const uint8_t cbuffer[LR1110_WIFI_GET_RESULT_SIZE_CMD_LENGTH] = {
        ( uint8_t )( LR1110_WIFI_GET_RESULT_SIZE_OC >> 8 ),
        ( uint8_t )( LR1110_WIFI_GET_RESULT_SIZE_OC & 0x00FF ),
    };

    return ( lr1110_status_t ) lr1110_hal_read( context, cbuffer, LR1110_WIFI_GET_RESULT_SIZE_CMD_LENGTH, nb_results,
                                                sizeof( *nb_results ) );
}

lr1110_status_t lr1110_wifi_read_basic_complete_results( const void* context, const uint8_t start_result_index,
                                                         const uint8_t                        nb_results,
                                                         lr1110_wifi_basic_complete_result_t* results )
{
    uint8_t       result_buffer[LR1110_WIFI_MAX_SIZE_PER_SPI( LR1110_WIFI_BASIC_COMPLETE_RESULT_SIZE )] = { 0 };
    const uint8_t nb_results_per_chunk_max =
        LR1110_WIFI_MAX_RESULT_PER_TRANSACTION( LR1110_WIFI_BASIC_COMPLETE_RESULT_SIZE );

    lr1110_wifi_result_interface_t result_interface = { 0 };
    result_interface.basic_complete                 = results;

    return fetch_and_aggregate_all_results( context, start_result_index, nb_results, nb_results_per_chunk_max,
                                            LR1110_WIFI_RESULT_FORMAT_BASIC_COMPLETE, result_buffer, result_interface );
}

lr1110_status_t lr1110_wifi_read_basic_mac_type_channel_results( const void* context, const uint8_t start_result_index,
                                                                 const uint8_t nb_results,
                                                                 lr1110_wifi_basic_mac_type_channel_result_t* results )
{
    uint8_t       result_buffer[LR1110_WIFI_MAX_SIZE_PER_SPI( LR1110_WIFI_BASIC_MAC_TYPE_CHANNEL_RESULT_SIZE )] = { 0 };
    const uint8_t nb_results_per_chunk_max =
        LR1110_WIFI_MAX_RESULT_PER_TRANSACTION( LR1110_WIFI_BASIC_MAC_TYPE_CHANNEL_RESULT_SIZE );

    lr1110_wifi_result_interface_t result_interface = { 0 };
    result_interface.basic_mac_type_channel         = results;

    return fetch_and_aggregate_all_results( context, start_result_index, nb_results, nb_results_per_chunk_max,
                                            LR1110_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL, result_buffer,
                                            result_interface );
}

lr1110_status_t lr1110_wifi_reset_cumulative_timing( const void* context )
{
    const uint8_t cbuffer[LR1110_WIFI_RESET_CUMUL_TIMING_CMD_LENGTH] = {
        ( uint8_t )( LR1110_WIFI_RESET_CUMUL_TIMING_OC >> 8 ), ( uint8_t )( LR1110_WIFI_RESET_CUMUL_TIMING_OC & 0x00FF )
    };
    return ( lr1110_status_t ) lr1110_hal_write( context, cbuffer, LR1110_WIFI_RESET_CUMUL_TIMING_CMD_LENGTH, 0, 0 );
}

lr1110_status_t lr1110_wifi_read_cumulative_timing( const void* context, lr1110_wifi_cumulative_timings_t* timing )
{
    const uint8_t cbuffer[LR1110_WIFI_READ_CUMUL_TIMING_CMD_LENGTH] = {
        ( uint8_t )( LR1110_WIFI_READ_CUMUL_TIMING_OC >> 8 ),
        ( uint8_t )( LR1110_WIFI_READ_CUMUL_TIMING_OC & 0x00FF ),
    };
    uint8_t buffer_out[LR1110_WIFI_ALL_CUMULATIVE_TIMING_SIZE] = { 0 };

    const lr1110_hal_status_t hal_status = lr1110_hal_read( context, cbuffer, LR1110_WIFI_READ_CUMUL_TIMING_CMD_LENGTH,
                                                            buffer_out, LR1110_WIFI_ALL_CUMULATIVE_TIMING_SIZE );

    if( hal_status == LR1110_HAL_STATUS_OK )
    {
        timing->rx_detection_us =
            ( buffer_out[0] << 24 ) + ( buffer_out[1] << 16 ) + ( buffer_out[2] << 8 ) + buffer_out[3];
        timing->rx_correlation_us =
            ( buffer_out[4] << 24 ) + ( buffer_out[5] << 16 ) + ( buffer_out[6] << 8 ) + buffer_out[7];
        timing->rx_capture_us =
            ( buffer_out[8] << 24 ) + ( buffer_out[9] << 16 ) + ( buffer_out[10] << 8 ) + buffer_out[11];
        timing->demodulation_us =
            ( buffer_out[12] << 24 ) + ( buffer_out[13] << 16 ) + ( buffer_out[14] << 8 ) + buffer_out[15];
    }
    return ( lr1110_status_t ) hal_status;
}

lr1110_status_t lr1110_wifi_get_nb_country_code_results( const void* context, uint8_t* country_result_size )
{
    const uint8_t cbuffer[LR1110_WIFI_GET_SIZE_COUNTRY_RESULT_CMD_LENGTH] = {
        ( uint8_t )( LR1110_WIFI_GET_SIZE_COUNTRY_RESULT_OC >> 8 ),
        ( uint8_t )( LR1110_WIFI_GET_SIZE_COUNTRY_RESULT_OC & 0x00FF ),
    };
    uint8_t rbuffer[LR1110_WIFI_COUNTRY_RESULT_LENGTH_SIZE] = { 0 };

    const lr1110_hal_status_t hal_status =
        lr1110_hal_read( context, cbuffer, LR1110_WIFI_GET_SIZE_COUNTRY_RESULT_CMD_LENGTH, rbuffer,
                         LR1110_WIFI_COUNTRY_RESULT_LENGTH_SIZE );
    if( hal_status == LR1110_HAL_STATUS_OK )
    {
        ( *country_result_size ) = rbuffer[0];
    }
    return ( lr1110_status_t ) hal_status;
}

lr1110_status_t lr1110_wifi_read_country_code_results( const void* context, const uint8_t start_result_index,
                                                       const uint8_t               nb_country_results,
                                                       lr1110_wifi_country_code_t* country_code_results )
{
    const uint8_t cbuffer[LR1110_WIFI_READ_COUNTRY_CODE_CMD_LENGTH] = {
        ( uint8_t )( LR1110_WIFI_READ_COUNTRY_CODE_OC >> 8 ), ( uint8_t )( LR1110_WIFI_READ_COUNTRY_CODE_OC & 0x00FF ),
        start_result_index, nb_country_results
    };
    uint8_t        rbuffer[LR1110_WIFI_MAX_COUNTRY_CODE_RESULT_SIZE] = { 0 };
    const uint16_t country_code_result_size_to_read =
        nb_country_results * LR1110_WIFI_SCAN_SINGLE_COUNTRY_CODE_RESULT_SIZE;

    const lr1110_hal_status_t hal_status = lr1110_hal_read( context, cbuffer, LR1110_WIFI_READ_COUNTRY_CODE_CMD_LENGTH,
                                                            rbuffer, country_code_result_size_to_read );

    if( hal_status == LR1110_HAL_STATUS_OK )
    {
        for( uint8_t result_index = 0; result_index < nb_country_results; result_index++ )
        {
            const uint8_t               local_index = result_index * LR1110_WIFI_SCAN_SINGLE_COUNTRY_CODE_RESULT_SIZE;
            lr1110_wifi_country_code_t* local_country_code_result = &country_code_results[result_index];

            local_country_code_result->country_code[0]   = rbuffer[local_index + 0];
            local_country_code_result->country_code[1]   = rbuffer[local_index + 1];
            local_country_code_result->io_regulation     = rbuffer[local_index + 2];
            local_country_code_result->channel_info_byte = rbuffer[local_index + 3];

            for( uint8_t field_mac_index = 0; field_mac_index < LR1110_WIFI_MAC_ADDRESS_LENGTH; field_mac_index++ )
            {
                local_country_code_result->mac_address[field_mac_index] =
                    rbuffer[local_index + ( LR1110_WIFI_MAC_ADDRESS_LENGTH - field_mac_index - 1 ) + 4];
            }
        }
    }
    return ( lr1110_status_t ) hal_status;
}

lr1110_status_t lr1110_wifi_cfg_timestamp_ap_phone( const void* context, uint32_t timestamp_in_s )
{
    uint8_t cbuffer[LR1110_WIFI_CONFIG_TIMESTAMP_AP_PHONE_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_WIFI_CONFIGURE_TIMESTAMP_AP_PHONE_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_WIFI_CONFIGURE_TIMESTAMP_AP_PHONE_OC >> 0 );

    cbuffer[2] = ( uint8_t )( timestamp_in_s >> 24 );
    cbuffer[3] = ( uint8_t )( timestamp_in_s >> 16 );
    cbuffer[4] = ( uint8_t )( timestamp_in_s >> 8 );
    cbuffer[5] = ( uint8_t )( timestamp_in_s >> 0 );

    return ( lr1110_status_t ) lr1110_hal_write( context, cbuffer, LR1110_WIFI_CONFIG_TIMESTAMP_AP_PHONE_CMD_LENGTH, 0,
                                                 0 );
}

lr1110_status_t lr1110_wifi_read_version( const void* context, lr1110_wifi_version_t* wifi_version )
{
    const uint8_t cbuffer[LR1110_WIFI_GET_VERSION_CMD_LENGTH] = {
        ( uint8_t )( LR1110_WIFI_GET_VERSION_OC >> 8 ),
        ( uint8_t )( LR1110_WIFI_GET_VERSION_OC & 0x00FF ),
    };
    uint8_t                   rbuffer[LR1110_WIFI_VERSION_SIZE] = { 0 };
    const lr1110_hal_status_t hal_status =
        lr1110_hal_read( context, cbuffer, LR1110_WIFI_GET_VERSION_CMD_LENGTH, rbuffer, LR1110_WIFI_VERSION_SIZE );
    if( hal_status == LR1110_HAL_STATUS_OK )
    {
        wifi_version->major = rbuffer[0];
        wifi_version->minor = rbuffer[1];
    }
    return ( lr1110_status_t ) hal_status;
}

uint8_t lr1110_wifi_n_results_max_per_chunk( void )
{
    return ( uint8_t ) LR1110_WIFI_N_RESULTS_MAX_PER_CHUNK;
}

#define CASE_CHANNEL( chan ) \
    case chan:               \
        return ( 1 << ( chan - 1 ) );
lr1110_wifi_channel_mask_t lr1110_wifi_channel_to_mask( const lr1110_wifi_channel_t channel )
{
    switch( channel )
    {
    case LR1110_WIFI_NO_CHANNEL:
        return ( lr1110_wifi_channel_mask_t ) 0x0000;
    case LR1110_WIFI_ALL_CHANNELS:
        return ( lr1110_wifi_channel_mask_t ) 0x3FFF;
        CASE_CHANNEL( LR1110_WIFI_CHANNEL_1 )
        CASE_CHANNEL( LR1110_WIFI_CHANNEL_2 )
        CASE_CHANNEL( LR1110_WIFI_CHANNEL_3 )
        CASE_CHANNEL( LR1110_WIFI_CHANNEL_4 )
        CASE_CHANNEL( LR1110_WIFI_CHANNEL_5 )
        CASE_CHANNEL( LR1110_WIFI_CHANNEL_6 )
        CASE_CHANNEL( LR1110_WIFI_CHANNEL_7 )
        CASE_CHANNEL( LR1110_WIFI_CHANNEL_8 )
        CASE_CHANNEL( LR1110_WIFI_CHANNEL_9 )
        CASE_CHANNEL( LR1110_WIFI_CHANNEL_10 )
        CASE_CHANNEL( LR1110_WIFI_CHANNEL_11 )
        CASE_CHANNEL( LR1110_WIFI_CHANNEL_12 )
        CASE_CHANNEL( LR1110_WIFI_CHANNEL_13 )
        CASE_CHANNEL( LR1110_WIFI_CHANNEL_14 )
    default:
        return ( lr1110_wifi_channel_mask_t ) 0x0000;
    }
}
#undef CASE_CHANNEL

void lr1110_extract_channel_info( const lr1110_wifi_channel_info_byte_t info_byte, lr1110_wifi_channel_t* channel,
                                  bool* rssi_validity, lr1110_wifi_mac_origin_t* mac_origin_estimation )
{
    ( *channel )               = lr1110_extract_channel_from_info_byte( info_byte );
    ( *mac_origin_estimation ) = ( lr1110_wifi_mac_origin_t )( ( info_byte & 0x30 ) >> 4 );
    ( *rssi_validity )         = ( ( info_byte & 0x40 ) == 0 ) ? true : false;
}

lr1110_wifi_channel_t lr1110_extract_channel_from_info_byte( const lr1110_wifi_channel_info_byte_t info_byte )
{
    return ( lr1110_wifi_channel_t )( info_byte & 0x0F );
}

void lr1110_extract_frame_type_info( const lr1110_wifi_frame_type_info_byte_t frame_type_info,
                                     lr1110_wifi_frame_type_t* frame_type, lr1110_wifi_frame_sub_type_t* frame_sub_type,
                                     bool* to_ds, bool* from_ds )
{
    ( *frame_type )     = ( lr1110_wifi_frame_type_t )( ( frame_type_info >> 6 ) & 0x03 );
    ( *frame_sub_type ) = ( lr1110_wifi_frame_sub_type_t )( ( frame_type_info >> 2 ) & 0x0F );
    ( *to_ds )          = ( bool ) ( ( frame_type_info >> 1 ) & 0x01 );
    ( *from_ds )        = ( bool ) ( frame_type_info & 0x01 );
}

void lr1110_extract_data_rate_info( const lr1110_wifi_datarate_info_byte_t data_rate_info,
                                    lr1110_wifi_signal_type_result_t*      wifi_signal_type,
                                    lr1110_wifi_datarate_t*                wifi_data_rate )
{
    ( *wifi_signal_type ) = lr1110_extract_signal_type_from_data_rate_info( data_rate_info );
    ( *wifi_data_rate )   = ( lr1110_wifi_datarate_t )( data_rate_info >> 2 );
}

lr1110_wifi_signal_type_result_t lr1110_extract_signal_type_from_data_rate_info(
    const lr1110_wifi_datarate_info_byte_t data_rate_info )
{
    return ( lr1110_wifi_signal_type_result_t )( data_rate_info & 0x03 );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static lr1110_hal_status_t lr1110_wifi_read_results_helper( const void* context, const uint8_t start_index,
                                                            const uint8_t n_elem, uint8_t* buffer,
                                                            const lr1110_wifi_result_format_t result_format )
{
    const uint8_t  size_single_elem   = lr1110_wifi_get_result_size_from_format( result_format );
    const uint8_t  result_format_code = lr1110_wifi_get_format_code( result_format );
    const uint8_t  cbuffer[LR1110_WIFI_READ_RESULT_CMD_LENGTH] = { ( uint8_t )( LR1110_WIFI_READ_RESULT_OC >> 8 ),
                                                                  ( uint8_t )( LR1110_WIFI_READ_RESULT_OC & 0x00FF ),
                                                                  start_index, n_elem, result_format_code };
    const uint16_t size_total                                  = n_elem * size_single_elem;
    return lr1110_hal_read( context, cbuffer, LR1110_WIFI_READ_RESULT_CMD_LENGTH, buffer, size_total );
}

static uint16_t uint16_from_array( const uint8_t* array, const uint16_t index )
{
    return ( uint16_t )( array[index] << 8 ) + ( ( uint16_t )( array[index + 1] ) );
}

static uint64_t uint64_from_array( const uint8_t* array, const uint16_t index )
{
    return ( ( uint64_t )( array[index] ) << 56 ) + ( ( uint64_t )( array[index + 1] ) << 48 ) +
           ( ( uint64_t )( array[index + 2] ) << 40 ) + ( ( uint64_t )( array[index + 3] ) << 32 ) +
           ( ( uint64_t )( array[index + 4] ) << 24 ) + ( ( uint64_t )( array[index + 5] ) << 16 ) +
           ( ( uint64_t )( array[index + 6] ) << 8 ) + ( uint64_t )( array[index + 7] );
}

static void lr1110_wifi_read_mac_address_from_buffer( const uint8_t* buffer, const uint16_t index_in_buffer,
                                                      lr1110_wifi_mac_address_t mac_address )
{
    for( uint8_t field_mac_index = 0; field_mac_index < LR1110_WIFI_MAC_ADDRESS_LENGTH; field_mac_index++ )
    {
        mac_address[field_mac_index] = buffer[index_in_buffer + field_mac_index];
    }
}

static uint8_t lr1110_wifi_get_format_code( const lr1110_wifi_result_format_t format )
{
    uint8_t format_code = 0x00;
    switch( format )
    {
    case LR1110_WIFI_RESULT_FORMAT_BASIC_COMPLETE:
    {
        format_code = 0x01;
        break;
    }
    case LR1110_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL:
    {
        format_code = 0x04;
        break;
    }
    }
    return format_code;
}

static uint8_t lr1110_wifi_get_result_size_from_format( const lr1110_wifi_result_format_t format )
{
    uint8_t result_size = 0;
    switch( format )
    {
    case LR1110_WIFI_RESULT_FORMAT_BASIC_COMPLETE:
    {
        result_size = LR1110_WIFI_BASIC_COMPLETE_RESULT_SIZE;
        break;
    }
    case LR1110_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL:
    {
        result_size = LR1110_WIFI_BASIC_MAC_TYPE_CHANNEL_RESULT_SIZE;
        break;
    }
    }
    return result_size;
}

static lr1110_status_t fetch_and_aggregate_all_results( const void* context, const uint8_t index_result_start,
                                                        const uint8_t                     nb_results,
                                                        const uint8_t                     nb_results_per_chunk_max,
                                                        const lr1110_wifi_result_format_t result_format_code,
                                                        uint8_t*                          result_buffer,
                                                        lr1110_wifi_result_interface_t    result_structures )
{
    uint8_t index_to_read     = index_result_start;
    uint8_t remaining_results = nb_results;

    lr1110_hal_status_t hal_status = LR1110_HAL_STATUS_OK;
    while( remaining_results > 0 )
    {
        uint8_t results_to_read = MIN( remaining_results, nb_results_per_chunk_max );

        lr1110_hal_status_t local_hal_status = lr1110_wifi_read_results_helper( context, index_to_read, results_to_read,
                                                                                result_buffer, result_format_code );
        if( local_hal_status != LR1110_HAL_STATUS_OK )
        {
            return ( lr1110_status_t ) local_hal_status;
        }

        generic_results_interpreter( results_to_read, result_buffer, result_structures, result_format_code );

        index_to_read += results_to_read;
        remaining_results -= results_to_read;
    }
    return ( lr1110_status_t ) hal_status;
}

static void generic_results_interpreter( const uint8_t n_result_to_parse, const uint8_t* buffer,
                                         lr1110_wifi_result_interface_t    result_interface,
                                         const lr1110_wifi_result_format_t format_code )
{
    switch( format_code )
    {
    case LR1110_WIFI_RESULT_FORMAT_BASIC_COMPLETE:
    {
        interpret_basic_complete_result_from_buffer( n_result_to_parse, buffer, result_interface.basic_complete );
        break;
    }

    case LR1110_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL:
    {
        interpret_basic_mac_type_channel_result_from_buffer( n_result_to_parse, buffer,
                                                             result_interface.basic_mac_type_channel );
        break;
    }
    }
}

static void interpret_basic_complete_result_from_buffer( const uint8_t nb_results, const uint8_t* buffer,
                                                         lr1110_wifi_basic_complete_result_t* result )
{
    for( uint8_t result_index = 0; result_index < nb_results; result_index++ )
    {
        const uint16_t                       local_index_start = LR1110_WIFI_BASIC_COMPLETE_RESULT_SIZE * result_index;
        lr1110_wifi_basic_complete_result_t* local_wifi_result = &result[result_index];
        local_wifi_result->data_rate_info_byte                 = buffer[local_index_start + 0];
        local_wifi_result->channel_info_byte                   = buffer[local_index_start + 1];
        local_wifi_result->rssi                                = buffer[local_index_start + 2];
        local_wifi_result->frame_type_info_byte                = buffer[local_index_start + 3];
        lr1110_wifi_read_mac_address_from_buffer( buffer, local_index_start + 4, local_wifi_result->mac_address );
        local_wifi_result->phi_offset       = uint16_from_array( buffer, local_index_start + 10 );
        local_wifi_result->timestamp_us     = uint64_from_array( buffer, local_index_start + 12 );
        local_wifi_result->beacon_period_tu = uint16_from_array( buffer, local_index_start + 20 );
    }
}

static void interpret_basic_mac_type_channel_result_from_buffer( const uint8_t nb_results, const uint8_t* buffer,
                                                                 lr1110_wifi_basic_mac_type_channel_result_t* result )
{
    for( uint8_t result_index = 0; result_index < nb_results; result_index++ )
    {
        const uint16_t local_index_start = LR1110_WIFI_BASIC_MAC_TYPE_CHANNEL_RESULT_SIZE * result_index;
        lr1110_wifi_basic_mac_type_channel_result_t* local_wifi_result = &result[result_index];
        local_wifi_result->data_rate_info_byte                         = buffer[local_index_start + 0];
        local_wifi_result->channel_info_byte                           = buffer[local_index_start + 1];
        local_wifi_result->rssi                                        = buffer[local_index_start + 2];
        lr1110_wifi_read_mac_address_from_buffer( buffer, local_index_start + 3, local_wifi_result->mac_address );
    }
}

/* --- EOF ------------------------------------------------------------------ */
