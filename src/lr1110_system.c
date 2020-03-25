/*!
 * \file      lr1110_system.c
 *
 * \brief     System driver implementation for LR1110
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

#include "lr1110_system.h"
#include "lr1110_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR1110_SYSTEM_GET_STATUS_CMD_LENGTH ( 2 + 4 )
#define LR1110_SYSTEM_GET_VERSION_CMD_LENGTH ( 2 )
#define LR1110_SYSTEM_GET_ERRORS_CMD_LENGTH ( 2 )
#define LR1110_SYSTEM_CLEAR_ERRORS_CMD_LENGTH ( 2 )
#define LR1110_SYSTEM_CALIBRATE_CMD_LENGTH ( 2 + 1 )
#define LR1110_SYSTEM_SET_REGMODE_CMD_LENGTH ( 2 + 1 )
#define LR1110_SYSTEM_CALIBRATE_IMAGE_CMD_LENGTH ( 2 + 2 )
#define LR1110_SYSTEM_SET_DIO_AS_RF_SWITCH_CMD_LENGTH ( 2 + 8 )
#define LR1110_SYSTEM_SET_DIO_IRQ_PARAMS_CMD_LENGTH ( 2 + 8 )
#define LR1110_SYSTEM_CLEAR_IRQ_CMD_LENGTH ( 2 + 4 )
#define LR1110_SYSTEM_CONFIG_LFCLK_CMD_LENGTH ( 2 + 1 )
#define LR1110_SYSTEM_SET_TCXO_MODE_CMD_LENGTH ( 2 + 4 )
#define LR1110_SYSTEM_REBOOT_CMD_LENGTH ( 2 + 1 )
#define LR1110_SYSTEM_GET_VBAT_CMD_LENGTH ( 2 )
#define LR1110_SYSTEM_GET_TEMP_CMD_LENGTH ( 2 )
#define LR1110_SYSTEM_SET_SLEEP_CMD_LENGTH ( 2 + 5 )
#define LR1110_SYSTEM_SET_STANDBY_CMD_LENGTH ( 2 + 1 )
#define LR1110_SYSTEM_SET_FS_CMD_LENGTH ( 2 )
#define LR1110_SYSTEM_ERASE_INFOPAGE_CMD_LENGTH ( 2 + 1 )
#define LR1110_SYSTEM_WRITE_INFOPAGE_CMD_LENGTH ( 2 + 3 )
#define LR1110_SYSTEM_READ_INFOPAGE_CMD_LENGTH ( 2 + 4 )
#define LR1110_SYSTEM_READ_UID_CMD_LENGTH ( 2 )
#define LR1110_SYSTEM_READ_JOIN_EUI_CMD_LENGTH ( 2 )
#define LR1110_SYSTEM_READ_PIN_CMD_LENGTH ( 2 )
#define LR1110_SYSTEM_GET_RANDOM_CMD_LENGTH ( 2 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

enum
{
    LR1110_SYSTEM_GET_STATUS_OC           = 0x0100,
    LR1110_SYSTEM_GET_VERSION_OC          = 0x0101,
    LR1110_SYSTEM_GET_ERRORS_OC           = 0x010D,
    LR1110_SYSTEM_CLEAR_ERRORS_OC         = 0x010E,
    LR1110_SYSTEM_CALIBRATE_OC            = 0x010F,
    LR1110_SYSTEM_SET_REGMODE_OC          = 0x0110,
    LR1110_SYSTEM_CALIBRATE_IMAGE_OC      = 0x0111,
    LR1110_SYSTEM_SET_DIO_AS_RF_SWITCH_OC = 0x0112,
    LR1110_SYSTEM_SET_DIOIRQPARAMS_OC     = 0x0113,
    LR1110_SYSTEM_CLEAR_IRQ_OC            = 0x0114,
    LR1110_SYSTEM_CONFIG_LFCLK_OC         = 0x0116,
    LR1110_SYSTEM_SET_TCXO_MODE_OC        = 0x0117,
    LR1110_SYSTEM_REBOOT_OC               = 0x0118,
    LR1110_SYSTEM_GET_VBAT_OC             = 0x0119,
    LR1110_SYSTEM_GET_TEMP_OC             = 0x011A,
    LR1110_SYSTEM_SET_SLEEP_OC            = 0x011B,
    LR1110_SYSTEM_SET_STANDBY_OC          = 0x011C,
    LR1110_SYSTEM_SET_FS_OC               = 0x011D,
    LR1110_SYSTEM_GET_RANDOM_OC           = 0x0120,
    LR1110_SYSTEM_ERASE_INFOPAGE_OC       = 0x0121,
    LR1110_SYSTEM_WRITE_INFOPAGE_OC       = 0x0122,
    LR1110_SYSTEM_READ_INFOPAGE_OC        = 0x0123,
    LR1110_SYSTEM_READ_UID_OC             = 0x0125,
    LR1110_SYSTEM_READ_JOIN_EUI_OC        = 0x0126,
    LR1110_SYSTEM_READ_PIN_OC             = 0x0127,
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

void lr1110_system_reset( const void* radio )
{
    lr1110_hal_reset( radio );
}

void lr1110_system_get_status( const void* radio, lr1110_system_stat1_t* stat1, lr1110_system_stat2_t* stat2,
                               uint32_t* irq_status )
{
    uint8_t cbuffer[LR1110_SYSTEM_GET_STATUS_CMD_LENGTH] = { 0x00 };

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_GET_STATUS_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_GET_STATUS_OC >> 0 );

    lr1110_hal_write_read( radio, cbuffer, cbuffer, LR1110_SYSTEM_GET_STATUS_CMD_LENGTH );

    stat1->is_interrupt_active = ( ( cbuffer[0] & 0x01 ) != 0 ) ? true : false;
    stat1->command_status      = ( lr1110_system_command_status_t )( cbuffer[0] >> 1 );

    stat2->is_running_from_flash = ( ( cbuffer[1] & 0x01 ) != 0 ) ? true : false;
    stat2->chip_mode             = ( lr1110_system_chip_mode_t )( cbuffer[1] >> 1 );

    *irq_status = ( ( uint32_t ) cbuffer[2] << 24 ) + ( ( uint32_t ) cbuffer[3] << 16 ) +
                  ( ( uint32_t ) cbuffer[4] << 8 ) + ( ( uint32_t ) cbuffer[5] << 0 );
}

void lr1110_system_get_version( const void* radio, lr1110_system_version_t* version )
{
    uint8_t cbuffer[LR1110_SYSTEM_GET_VERSION_CMD_LENGTH];
    uint8_t rbuffer[sizeof( lr1110_system_version_t )] = { 0x00 };

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_GET_VERSION_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_GET_VERSION_OC >> 0 );

    lr1110_hal_read( radio, cbuffer, LR1110_SYSTEM_GET_VERSION_CMD_LENGTH, rbuffer, sizeof( lr1110_system_version_t ) );

    version->hw   = rbuffer[0];
    version->type = rbuffer[1];
    version->fw   = ( ( uint16_t ) rbuffer[2] << 8 ) + ( uint16_t ) rbuffer[3];
}

void lr1110_system_get_errors( const void* radio, lr1110_system_errors_t* errors )
{
    uint8_t cbuffer[LR1110_SYSTEM_GET_ERRORS_CMD_LENGTH];
    uint8_t rbuffer[sizeof( errors )] = { 0x00 };

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_GET_ERRORS_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_GET_ERRORS_OC >> 0 );

    lr1110_hal_read( radio, cbuffer, LR1110_SYSTEM_GET_ERRORS_CMD_LENGTH, rbuffer, sizeof( *errors ) );

    *errors = ( ( uint16_t ) rbuffer[0] << 8 ) + ( uint16_t ) rbuffer[1];
}

void lr1110_system_clear_errors( const void* radio )
{
    uint8_t cbuffer[LR1110_SYSTEM_CLEAR_ERRORS_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_CLEAR_ERRORS_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_CLEAR_ERRORS_OC >> 0 );

    lr1110_hal_write( radio, cbuffer, LR1110_SYSTEM_CLEAR_ERRORS_CMD_LENGTH, 0, 0 );
}

void lr1110_system_calibrate( const void* radio, const uint8_t calib_params )
{
    uint8_t cbuffer[LR1110_SYSTEM_CALIBRATE_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_CALIBRATE_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_CALIBRATE_OC >> 0 );

    cbuffer[2] = calib_params;

    lr1110_hal_write( radio, cbuffer, LR1110_SYSTEM_CALIBRATE_CMD_LENGTH, 0, 0 );
}

void lr1110_system_set_regmode( const void* radio, const lr1110_regmodes_t reg_mode )
{
    uint8_t cbuffer[LR1110_SYSTEM_SET_REGMODE_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_SET_REGMODE_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_SET_REGMODE_OC >> 0 );

    cbuffer[2] = ( uint8_t ) reg_mode;

    lr1110_hal_write( radio, cbuffer, LR1110_SYSTEM_SET_REGMODE_CMD_LENGTH, 0, 0 );
}

void lr1110_system_calibrate_image( const void* radio, const uint8_t freq1, const uint8_t freq2 )
{
    uint8_t cbuffer[LR1110_SYSTEM_CALIBRATE_IMAGE_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_CALIBRATE_IMAGE_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_CALIBRATE_IMAGE_OC >> 0 );

    cbuffer[2] = freq1;
    cbuffer[3] = freq2;

    lr1110_hal_write( radio, cbuffer, LR1110_SYSTEM_CALIBRATE_IMAGE_CMD_LENGTH, 0, 0 );
}

void lr1110_system_set_dio_as_rf_switch( const void*                            radio,
                                         const lr1110_system_rfswitch_config_t* rf_switch_configuration )
{
    uint8_t cbuffer[LR1110_SYSTEM_SET_DIO_AS_RF_SWITCH_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_SET_DIO_AS_RF_SWITCH_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_SET_DIO_AS_RF_SWITCH_OC >> 0 );

    cbuffer[2] = rf_switch_configuration->enable;
    cbuffer[3] = rf_switch_configuration->standby;
    cbuffer[4] = rf_switch_configuration->rx;
    cbuffer[5] = rf_switch_configuration->tx;
    cbuffer[6] = rf_switch_configuration->tx_hp;
    cbuffer[7] = rf_switch_configuration->tx_hf;
    cbuffer[8] = rf_switch_configuration->gnss;
    cbuffer[9] = rf_switch_configuration->wifi;

    lr1110_hal_write( radio, cbuffer, LR1110_SYSTEM_SET_DIO_AS_RF_SWITCH_CMD_LENGTH, 0, 0 );
}

void lr1110_system_set_dio_irq_params( const void* radio, const uint32_t irqs_to_enable_dio1,
                                       const uint32_t irqs_to_enable_dio2 )
{
    uint8_t cbuffer[LR1110_SYSTEM_SET_DIO_IRQ_PARAMS_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_SET_DIOIRQPARAMS_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_SET_DIOIRQPARAMS_OC >> 0 );

    cbuffer[2] = ( uint8_t )( irqs_to_enable_dio1 >> 24 );
    cbuffer[3] = ( uint8_t )( irqs_to_enable_dio1 >> 16 );
    cbuffer[4] = ( uint8_t )( irqs_to_enable_dio1 >> 8 );
    cbuffer[5] = ( uint8_t )( irqs_to_enable_dio1 >> 0 );

    cbuffer[6] = ( uint8_t )( irqs_to_enable_dio2 >> 24 );
    cbuffer[7] = ( uint8_t )( irqs_to_enable_dio2 >> 16 );
    cbuffer[8] = ( uint8_t )( irqs_to_enable_dio2 >> 8 );
    cbuffer[9] = ( uint8_t )( irqs_to_enable_dio2 >> 0 );

    lr1110_hal_write( radio, cbuffer, LR1110_SYSTEM_SET_DIO_IRQ_PARAMS_CMD_LENGTH, 0, 0 );
}

void lr1110_system_clear_irq( const void* radio, const uint32_t irqs_to_clear )
{
    uint8_t cbuffer[LR1110_SYSTEM_CLEAR_IRQ_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_CLEAR_IRQ_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_CLEAR_IRQ_OC >> 0 );

    cbuffer[2] = ( uint8_t )( irqs_to_clear >> 24 );
    cbuffer[3] = ( uint8_t )( irqs_to_clear >> 16 );
    cbuffer[4] = ( uint8_t )( irqs_to_clear >> 8 );
    cbuffer[5] = ( uint8_t )( irqs_to_clear >> 0 );

    lr1110_hal_write( radio, cbuffer, LR1110_SYSTEM_CLEAR_IRQ_CMD_LENGTH, 0, 0 );
}

void lr1110_system_config_lfclk( const void* radio, const lr1110_system_lfclk_config_t lfclock_config,
                                 const bool wait_for_32k_ready )
{
    uint8_t cbuffer[LR1110_SYSTEM_CONFIG_LFCLK_CMD_LENGTH];
    uint8_t config = lfclock_config | ( wait_for_32k_ready << 2 );

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_CONFIG_LFCLK_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_CONFIG_LFCLK_OC >> 0 );

    cbuffer[2] = config;

    lr1110_hal_write( radio, cbuffer, LR1110_SYSTEM_CONFIG_LFCLK_CMD_LENGTH, 0, 0 );
}

void lr1110_system_set_tcxo_mode( const void* radio, const lr1110_system_tcxo_supply_voltage_t tune,
                                  const uint32_t timeout )
{
    uint8_t cbuffer[LR1110_SYSTEM_SET_TCXO_MODE_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_SET_TCXO_MODE_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_SET_TCXO_MODE_OC >> 0 );

    cbuffer[2] = ( uint8_t ) tune;

    cbuffer[3] = ( uint8_t )( timeout >> 16 );
    cbuffer[4] = ( uint8_t )( timeout >> 8 );
    cbuffer[5] = ( uint8_t )( timeout >> 0 );

    lr1110_hal_write( radio, cbuffer, LR1110_SYSTEM_SET_TCXO_MODE_CMD_LENGTH, 0, 0 );
}

void lr1110_system_reboot( const void* radio, const bool stay_in_bootloader )
{
    uint8_t cbuffer[LR1110_SYSTEM_REBOOT_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_REBOOT_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_REBOOT_OC >> 0 );

    cbuffer[2] = ( stay_in_bootloader == true ) ? 0x03 : 0x00;

    lr1110_hal_write( radio, cbuffer, LR1110_SYSTEM_REBOOT_CMD_LENGTH, 0, 0 );
}

void lr1110_system_get_vbat( const void* radio, uint8_t* vbat )
{
    uint8_t cbuffer[LR1110_SYSTEM_GET_VBAT_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_GET_VBAT_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_GET_VBAT_OC >> 0 );

    lr1110_hal_read( radio, cbuffer, LR1110_SYSTEM_GET_VBAT_CMD_LENGTH, vbat, sizeof( *vbat ) );
}

void lr1110_system_get_temp( const void* radio, uint16_t* temp )
{
    uint8_t cbuffer[LR1110_SYSTEM_GET_TEMP_CMD_LENGTH];
    uint8_t rbuffer[sizeof( uint16_t )] = { 0x00 };

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_GET_TEMP_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_GET_TEMP_OC >> 0 );

    lr1110_hal_read( radio, cbuffer, LR1110_SYSTEM_GET_TEMP_CMD_LENGTH, rbuffer, sizeof( uint16_t ) );

    *temp = ( ( uint16_t ) rbuffer[0] << 8 ) + ( uint16_t ) rbuffer[1];
}

void lr1110_system_set_sleep( const void* radio, const lr1110_system_sleep_config_t sleep_config,
                              const uint32_t sleep_time )
{
    uint8_t cbuffer[LR1110_SYSTEM_SET_SLEEP_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_SET_SLEEP_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_SET_SLEEP_OC >> 0 );

    cbuffer[2] = ( sleep_config.is_rtc_timeout << 1 ) + sleep_config.is_warm_start;

    cbuffer[3] = ( uint8_t )( sleep_time >> 24 );
    cbuffer[4] = ( uint8_t )( sleep_time >> 16 );
    cbuffer[5] = ( uint8_t )( sleep_time >> 8 );
    cbuffer[6] = ( uint8_t )( sleep_time >> 0 );

    lr1110_hal_write( radio, cbuffer, LR1110_SYSTEM_SET_SLEEP_CMD_LENGTH, 0, 0 );
}

void lr1110_system_set_standby( const void* radio, const lr1110_system_standby_config_t standby_config )
{
    uint8_t cbuffer[LR1110_SYSTEM_SET_STANDBY_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_SET_STANDBY_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_SET_STANDBY_OC >> 0 );

    cbuffer[2] = ( uint8_t ) standby_config;

    lr1110_hal_write( radio, cbuffer, LR1110_SYSTEM_SET_STANDBY_CMD_LENGTH, 0, 0 );
}

void lr1110_system_set_fs( const void* radio )
{
    uint8_t cbuffer[LR1110_SYSTEM_SET_FS_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_SET_FS_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_SET_FS_OC >> 0 );

    lr1110_hal_write( radio, cbuffer, LR1110_SYSTEM_SET_FS_CMD_LENGTH, 0, 0 );
}

void lr1110_system_erase_infopage( const void* radio, const lr1110_system_infopage_id_t infopage_id )
{
    uint8_t cbuffer[LR1110_SYSTEM_ERASE_INFOPAGE_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_ERASE_INFOPAGE_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_ERASE_INFOPAGE_OC >> 0 );

    cbuffer[2] = ( uint8_t ) infopage_id;

    lr1110_hal_write( radio, cbuffer, LR1110_SYSTEM_ERASE_INFOPAGE_CMD_LENGTH, 0, 0 );
}

void lr1110_system_write_infopage( const void* radio, const lr1110_system_infopage_id_t infopage_id,
                                   const uint16_t address, const uint32_t* data, const uint8_t length )
{
    uint8_t cbuffer[LR1110_SYSTEM_WRITE_INFOPAGE_CMD_LENGTH];
    uint8_t cdata[256];

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_WRITE_INFOPAGE_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_WRITE_INFOPAGE_OC >> 0 );

    cbuffer[2] = ( uint8_t ) infopage_id;

    cbuffer[3] = ( uint8_t )( address >> 8 );
    cbuffer[4] = ( uint8_t )( address >> 0 );

    for( uint16_t index = 0; index < length; index++ )
    {
        uint8_t* cdata_local = &cdata[index * sizeof( uint32_t )];

        cdata_local[0] = ( uint8_t )( data[index] >> 24 );
        cdata_local[1] = ( uint8_t )( data[index] >> 16 );
        cdata_local[2] = ( uint8_t )( data[index] >> 8 );
        cdata_local[3] = ( uint8_t )( data[index] >> 0 );
    }

    lr1110_hal_write( radio, cbuffer, LR1110_SYSTEM_WRITE_INFOPAGE_CMD_LENGTH, cdata, length * sizeof( uint32_t ) );
}

void lr1110_system_read_infopage( const void* radio, const lr1110_system_infopage_id_t infopage_id,
                                  const uint16_t address, uint32_t* data, const uint8_t length )
{
    uint8_t cbuffer[LR1110_SYSTEM_READ_INFOPAGE_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_READ_INFOPAGE_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_READ_INFOPAGE_OC >> 0 );

    cbuffer[2] = ( uint8_t ) infopage_id;

    cbuffer[3] = ( uint8_t )( address >> 8 );
    cbuffer[4] = ( uint8_t )( address >> 0 );

    cbuffer[5] = length;

    lr1110_hal_read( radio, cbuffer, LR1110_SYSTEM_READ_INFOPAGE_CMD_LENGTH, ( uint8_t* ) data,
                     length * sizeof( *data ) );

    for( uint8_t index = 0; index < length; index++ )
    {
        uint8_t* buffer_local = ( uint8_t* ) &data[index];

        data[index] = ( ( uint32_t ) buffer_local[0] << 24 ) + ( ( uint32_t ) buffer_local[1] << 16 ) +
                      ( ( uint32_t ) buffer_local[2] << 8 ) + ( ( uint32_t ) buffer_local[3] << 0 );
    }
}

void lr1110_system_read_uid( const void* radio, lr1110_system_uid_t unique_identifier )
{
    uint8_t cbuffer[LR1110_SYSTEM_READ_UID_CMD_LENGTH];
    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_READ_UID_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_READ_UID_OC >> 0 );

    lr1110_hal_read( radio, cbuffer, LR1110_SYSTEM_READ_UID_CMD_LENGTH, unique_identifier, LR1110_SYSTEM_UID_LENGTH );
}

void lr1110_system_read_join_eui( const void* radio, lr1110_system_join_eui_t join_eui )
{
    uint8_t cbuffer[LR1110_SYSTEM_READ_UID_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_READ_JOIN_EUI_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_READ_JOIN_EUI_OC >> 0 );

    lr1110_hal_read( radio, cbuffer, LR1110_SYSTEM_READ_JOIN_EUI_CMD_LENGTH, join_eui, LR1110_SYSTEM_JOIN_EUI_LENGTH );
}

void lr1110_system_read_pin( const void* radio, lr1110_system_pin_t pin )
{
    uint8_t cbuffer[LR1110_SYSTEM_READ_UID_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_READ_PIN_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_READ_PIN_OC >> 0 );

    lr1110_hal_read( radio, cbuffer, LR1110_SYSTEM_READ_PIN_CMD_LENGTH, pin, LR1110_SYSTEM_PIN_LENGTH );
}

void lr1110_system_get_random_number( const void* radio, uint32_t* random_number )
{
    uint8_t cbuffer[LR1110_SYSTEM_GET_RANDOM_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_SYSTEM_GET_RANDOM_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_SYSTEM_GET_RANDOM_OC >> 0 );

    lr1110_hal_read( radio, cbuffer, LR1110_SYSTEM_GET_RANDOM_CMD_LENGTH, ( uint8_t* ) random_number,
                     sizeof( uint32_t ) );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
