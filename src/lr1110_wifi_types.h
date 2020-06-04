/*!
 * \file      lr1110_wifi_types.h
 *
 * \brief     Wi-Fi passive scan driver types for LR1110
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

#ifndef __LR1110_WIFI_TYPES_H__
#define __LR1110_WIFI_TYPES_H__

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

#define LR1110_WIFI_MAC_ADDRESS_LENGTH ( 6 )
#define LR1110_WIFI_MAX_RESULTS ( 32 )
#define LR1110_WIFI_RESULT_SSID_LENGTH ( 32 )
#define LR1110_WIFI_MAX_COUNTRY_CODE ( 32 )
#define LR1110_WIFI_STR_COUNTRY_CODE_SIZE ( 2 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef uint16_t lr1110_wifi_channel_mask_t;

typedef uint8_t lr1110_wifi_channel_info_byte_t;

typedef uint8_t lr1110_wifi_datarate_info_byte_t;

typedef uint8_t lr1110_wifi_frame_type_info_byte_t;

typedef uint8_t lr1110_wifi_frame_sub_type_t;

typedef struct lr1110_wifi_fcs_info_byte_s
{
    bool is_fcs_ok;       //!< True if the LR1110 has checked the FCS and the check
                          //!< succeeded
    bool is_fcs_checked;  //!< True if the LR1110 has checked the FCS
} lr1110_wifi_fcs_info_byte_t;

/*!
 * \brief Type to store a MAC address
 */
typedef uint8_t lr1110_wifi_mac_address_t[LR1110_WIFI_MAC_ADDRESS_LENGTH];

/*!
 * \brief Wi-Fi Channels index
 */
typedef enum
{
    LR1110_WIFI_NO_CHANNEL   = 0x00,
    LR1110_WIFI_CHANNEL_1    = 0x01,  //!< Channel at frequency 2.412 GHz
    LR1110_WIFI_CHANNEL_2    = 0x02,  //!< Channel at frequency 2.417 GHz
    LR1110_WIFI_CHANNEL_3    = 0x03,  //!< Channel at frequency 2.422 GHz
    LR1110_WIFI_CHANNEL_4    = 0x04,  //!< Channel at frequency 2.427 GHz
    LR1110_WIFI_CHANNEL_5    = 0x05,  //!< Channel at frequency 2.432 GHz
    LR1110_WIFI_CHANNEL_6    = 0x06,  //!< Channel at frequency 2.437 GHz
    LR1110_WIFI_CHANNEL_7    = 0x07,  //!< Channel at frequency 2.442 GHz
    LR1110_WIFI_CHANNEL_8    = 0x08,  //!< Channel at frequency 2.447 GHz
    LR1110_WIFI_CHANNEL_9    = 0x09,  //!< Channel at frequency 2.452 GHz
    LR1110_WIFI_CHANNEL_10   = 0x0A,  //!< Channel at frequency 2.457 GHz
    LR1110_WIFI_CHANNEL_11   = 0x0B,  //!< Channel at frequency 2.462 GHz
    LR1110_WIFI_CHANNEL_12   = 0x0C,  //!< Channel at frequency 2.467 GHz
    LR1110_WIFI_CHANNEL_13   = 0x0D,  //!< Channel at frequency 2.472 GHz
    LR1110_WIFI_CHANNEL_14   = 0x0E,  //!< Channel at frequency 2.484 GHz
    LR1110_WIFI_ALL_CHANNELS = 0x0F,
} lr1110_wifi_channel_t;

/*!
 * \brief WiFi theoretical Datarates
 */
typedef enum
{
    LR1110_WIFI_DATARATE_1_MBPS    = 1,
    LR1110_WIFI_DATARATE_2_MBPS    = 2,
    LR1110_WIFI_DATARATE_6_MBPS    = 3,
    LR1110_WIFI_DATARATE_9_MBPS    = 4,
    LR1110_WIFI_DATARATE_12_MBPS   = 5,
    LR1110_WIFI_DATARATE_18_MBPS   = 6,
    LR1110_WIFI_DATARATE_24_MBPS   = 7,
    LR1110_WIFI_DATARATE_36_MBPS   = 8,
    LR1110_WIFI_DATARATE_48_MBPS   = 9,
    LR1110_WIFI_DATARATE_54_MBPS   = 10,
    LR1110_WIFI_DATARATE_6_5_MBPS  = 11,
    LR1110_WIFI_DATARATE_13_MBPS   = 12,
    LR1110_WIFI_DATARATE_19_5_MBPS = 13,
    LR1110_WIFI_DATARATE_26_MBPS   = 14,
    LR1110_WIFI_DATARATE_39_MBPS   = 15,
    LR1110_WIFI_DATARATE_52_MBPS   = 16,
    LR1110_WIFI_DATARATE_58_MBPS   = 17,
    LR1110_WIFI_DATARATE_65_MBPS   = 18,
    LR1110_WIFI_DATARATE_7_2_MBPS  = 19,
    LR1110_WIFI_DATARATE_14_4_MBPS = 20,
    LR1110_WIFI_DATARATE_21_7_MBPS = 21,
    LR1110_WIFI_DATARATE_28_9_MBPS = 22,
    LR1110_WIFI_DATARATE_43_3_MBPS = 23,
    LR1110_WIFI_DATARATE_57_8_MBPS = 24,
    LR1110_WIFI_DATARATE_65_2_MBPS = 25,
    LR1110_WIFI_DATARATE_72_2_MBPS = 26,
} lr1110_wifi_datarate_t;

/*!
 * \brief WiFi Frame Types
 */
typedef enum
{
    LR1110_WIFI_FRAME_TYPE_MANAGEMENT = 0x00,
    LR1110_WIFI_FRAME_TYPE_CONTROL    = 0x01,
    LR1110_WIFI_FRAME_TYPE_DATA       = 0x02,
} lr1110_wifi_frame_type_t;

/*!
 * \brief The WiFi MAC address origin
 *
 * See lr1110_extract_channel_info for details about the MAC address origin
 * estimation of the LR1110.
 *
 * \see lr1110_extract_channel_info
 */
typedef enum
{
    LR1110_WIFI_ORIGIN_BEACON_FIX_AP    = 1,
    LR1110_WIFI_ORIGIN_BEACON_MOBILE_AP = 2,
    LR1110_WIFI_ORIGIN_PACKET           = 3,
} lr1110_wifi_mac_origin_t;

/*!
 * \brief Wi-Fi signal type for passive scanning configuration
 *
 * Note it is not possible to configure the WiFi passive scanning to search
 * Wi-Fi type N GreenField. Only Wi-Fi type N Mixed Mode can be scanned by
 * LR1110.
 *
 * \warning LR1110_WIFI_TYPE_SCAN_G and LR1110_WIFI_TYPE_SCAN_N configurations
 * are implemented the same way, and both will scan Wi-Fi type G **AND** Wi-Fi
 * type N.
 */
typedef enum
{
    LR1110_WIFI_TYPE_SCAN_B     = 0x01,  //!< WiFi B
    LR1110_WIFI_TYPE_SCAN_G     = 0x02,  //!< WiFi G
    LR1110_WIFI_TYPE_SCAN_N     = 0x03,  //!< WiFi N
    LR1110_WIFI_TYPE_SCAN_B_G_N = 0x04,  //!< Scan WiFi B and WiFi G/N
} lr1110_wifi_signal_type_scan_t;

/*!
 * \brief Wi-Fi signal type for passive scan results
 *
 * Note that the WiFi N detected is WiFi N Mixed mode, and not GreenField.
 */
typedef enum
{
    LR1110_WIFI_TYPE_RESULT_B = 0x01,  //!< WiFi B
    LR1110_WIFI_TYPE_RESULT_G = 0x02,  //!< WiFi G
    LR1110_WIFI_TYPE_RESULT_N = 0x03,  //!< WiFi N
} lr1110_wifi_signal_type_result_t;

/*!
 * \brief WiFi capture mode
 */
typedef enum
{
    LR1110_WIFI_SCAN_MODE_BEACON = 1,             //!< Only scan for Beacons and Probe Response Access Points' MAC
                                                  //!< addresses until Period Beacon field (Basic result)
    LR1110_WIFI_SCAN_MODE_BEACON_AND_PACKET = 2,  //!< Scan for beacons Access
                                                  //!< Points' MAC addresses until Period Beacon field, and for
                                                  //!< Packets until third Mac Address field (Basic result)
} lr1110_wifi_mode_t;

/*!
 * \brief Cumulative timings
 *
 * This structure is representing the cumulative time spent in the different
 * modes of Wi-Fi passive scanning procedure. Timing provided in [us].
 * */
typedef struct lr1110_wifi_cumulative_timings_s
{
    uint32_t rx_detection_us;    //!< Cumulative time spent during NFE or TOA
    uint32_t rx_correlation_us;  //!< Cumulative time spent during preamble detection
    uint32_t rx_capture_us;      //!< Cumulative time spent during signal acquisition
    uint32_t demodulation_us;    //!< Cumulative time spent during software
                                 //!< demodulation
} lr1110_wifi_cumulative_timings_t;

/*!
 * \brief Basic complete result structure
 *
 * The beacon period is expressed in TU (Time Unit). 1 TU is 1024 microseconds.
 */
typedef struct lr1110_wifi_basic_complete_result_s
{
    lr1110_wifi_datarate_info_byte_t   data_rate_info_byte;
    lr1110_wifi_channel_info_byte_t    channel_info_byte;
    int8_t                             rssi;
    lr1110_wifi_frame_type_info_byte_t frame_type_info_byte;
    lr1110_wifi_mac_address_t          mac_address;
    int16_t                            phi_offset;
    uint64_t                           timestamp_us;  //!< Indicate the up-time of the Access Point
                                                      //!< transmitting the Beacon [us]
    uint16_t beacon_period_tu;
} lr1110_wifi_basic_complete_result_t;

/*!
 * \brief Basic MAC, type, channel result structure
 */
typedef struct lr1110_wifi_basic_mac_type_channel_result_s
{
    lr1110_wifi_datarate_info_byte_t data_rate_info_byte;
    lr1110_wifi_channel_info_byte_t  channel_info_byte;
    int8_t                           rssi;
    lr1110_wifi_mac_address_t        mac_address;
} lr1110_wifi_basic_mac_type_channel_result_t;

typedef enum
{
    LR1110_WIFI_RESULT_FORMAT_BASIC_COMPLETE,
    LR1110_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL,
} lr1110_wifi_result_format_t;

typedef struct lr1110_wifi_country_code_s
{
    uint8_t                         country_code[LR1110_WIFI_STR_COUNTRY_CODE_SIZE];
    uint8_t                         io_regulation;  //!< Input Output Regulation
    lr1110_wifi_channel_info_byte_t channel_info_byte;
    lr1110_wifi_mac_address_t       mac_address;
} lr1110_wifi_country_code_t;

typedef struct lr1110_wifi_version_s
{
    uint8_t major;
    uint8_t minor;
} lr1110_wifi_version_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // __LR1110_WIFI_TYPES_H__

/* --- EOF ------------------------------------------------------------------ */
