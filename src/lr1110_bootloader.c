/*!
 * \file      lr1110_bootloader.c
 *
 * \brief     Bootloader driver implementation for LR1110
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

#include "lr1110_bootloader.h"
#include "lr1110_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR1110_FLASH_DATA_UINT32_MAX ( 64 )
#define LR1110_FLASH_DATA_UINT8_MAX ( LR1110_FLASH_DATA_UINT32_MAX * 4 )

#define LR1110_BL_CMD_NO_PARAM_LENGTH 2
#define LR1110_BL_VERSION_CMD_LENGTH LR1110_BL_CMD_NO_PARAM_LENGTH
#define LR1110_BL_ERASE_FLASH_CMD_LENGTH LR1110_BL_CMD_NO_PARAM_LENGTH
#define LR1110_BL_ERASE_PAGE_CMD_LENGTH ( LR1110_BL_CMD_NO_PARAM_LENGTH + 1 )
#define LR1110_BL_WRITE_FLASH_CMD_LENGTH ( LR1110_BL_CMD_NO_PARAM_LENGTH + 4 )
#define LR1110_BL_WRITE_FLASH_ENCRYPTED_CMD_LENGTH ( LR1110_BL_CMD_NO_PARAM_LENGTH + 4 )
#define LR1110_BL_GET_HASH_CMD_LENGTH ( LR1110_BL_CMD_NO_PARAM_LENGTH )
#define LR1110_BL_REBOOT_CMD_LENGTH ( LR1110_BL_CMD_NO_PARAM_LENGTH + 1 )
#define LR1110_BL_GET_PIN_CMD_LENGTH ( LR1110_BL_CMD_NO_PARAM_LENGTH )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

enum
{
    LR1110_BL_GET_STATUS_OC            = 0x0100,
    LR1110_BL_GET_VERSION_OC           = 0x0101,
    LR1110_BL_ERASE_FLASH_OC           = 0x8000,
    LR1110_BL_ERASE_PAGE_OC            = 0x8001,
    LR1110_BL_WRITE_FLASH_OC           = 0x8002,
    LR1110_BL_WRITE_FLASH_ENCRYPTED_OC = 0x8003,
    LR1110_BL_GET_HASH_OC              = 0x8004,
    LR1110_BL_REBOOT_OC                = 0x8005,
    LR1110_BL_GET_PIN_OC               = 0x800B,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

uint32_t min( uint32_t a, uint32_t b )
{
    uint32_t min = a;

    if( a > b )
    {
        min = b;
    }

    return min;
}

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void lr1110_bootloader_get_version( const void* radio, lr1110_bootloader_version_t* version )
{
    uint8_t cbuffer[LR1110_BL_VERSION_CMD_LENGTH];
    uint8_t rbuffer[LR1110_BL_VERSION_LENGTH] = { 0x00 };

    cbuffer[0] = ( uint8_t )( LR1110_BL_GET_VERSION_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_BL_GET_VERSION_OC >> 0 );

    lr1110_hal_read( radio, cbuffer, LR1110_BL_VERSION_CMD_LENGTH, rbuffer, LR1110_BL_VERSION_LENGTH );

    version->version_hw = rbuffer[0];
    version->version_fw = rbuffer[1];
}

void lr1110_bootloader_erase_flash( const void* radio )
{
    uint8_t cbuffer[LR1110_BL_ERASE_FLASH_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_BL_ERASE_FLASH_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_BL_ERASE_FLASH_OC >> 0 );

    lr1110_hal_write( radio, cbuffer, LR1110_BL_ERASE_FLASH_CMD_LENGTH, 0, 0 );
}

void lr1110_bootloader_erase_page( const void* radio, const uint8_t page_number )
{
    uint8_t cbuffer[LR1110_BL_ERASE_PAGE_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_BL_ERASE_PAGE_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_BL_ERASE_PAGE_OC >> 0 );

    cbuffer[2] = page_number;

    lr1110_hal_write( radio, cbuffer, LR1110_BL_ERASE_PAGE_CMD_LENGTH, 0, 0 );
}

void lr1110_bootloader_write_flash( const void* radio, const uint32_t offset, const uint32_t* data, uint8_t length )
{
    uint8_t cbuffer[LR1110_BL_WRITE_FLASH_CMD_LENGTH];
    uint8_t cdata[256];

    cbuffer[0] = ( uint8_t )( LR1110_BL_WRITE_FLASH_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_BL_WRITE_FLASH_OC >> 0 );

    cbuffer[2] = ( uint8_t )( offset >> 24 );
    cbuffer[3] = ( uint8_t )( offset >> 16 );
    cbuffer[4] = ( uint8_t )( offset >> 8 );
    cbuffer[5] = ( uint8_t )( offset >> 0 );

    for( uint8_t i = 0; i < length; i++ )
    {
        cdata[i * sizeof( uint32_t ) + 0] = ( uint8_t )( data[i] >> 24 );
        cdata[i * sizeof( uint32_t ) + 1] = ( uint8_t )( data[i] >> 16 );
        cdata[i * sizeof( uint32_t ) + 2] = ( uint8_t )( data[i] >> 8 );
        cdata[i * sizeof( uint32_t ) + 3] = ( uint8_t )( data[i] >> 0 );
    }

    lr1110_hal_write( radio, cbuffer, LR1110_BL_WRITE_FLASH_CMD_LENGTH, cdata, length * sizeof( uint32_t ) );
}

void lr1110_bootloader_write_flash_full( const void* radio, const uint32_t offset, const uint32_t* buffer,
                                         const uint32_t length )
{
    uint32_t remaining_length = length;
    uint32_t local_offset     = offset;
    uint32_t loop             = 0;

    while( remaining_length != 0 )
    {
        lr1110_bootloader_write_flash( radio, local_offset, buffer + loop * 64, min( remaining_length, 64 ) );

        local_offset += LR1110_FLASH_DATA_UINT8_MAX;
        remaining_length = ( remaining_length < 64 ) ? 0 : ( remaining_length - 64 );

        loop++;
    }
}

void lr1110_bootloader_write_flash_encrypted( const void* radio, const uint32_t offset, const uint32_t* data,
                                              uint8_t length )
{
    uint8_t cbuffer[LR1110_BL_WRITE_FLASH_ENCRYPTED_CMD_LENGTH];
    uint8_t cdata[256];

    cbuffer[0] = ( uint8_t )( LR1110_BL_WRITE_FLASH_ENCRYPTED_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_BL_WRITE_FLASH_ENCRYPTED_OC >> 0 );

    cbuffer[2] = ( uint8_t )( offset >> 24 );
    cbuffer[3] = ( uint8_t )( offset >> 16 );
    cbuffer[4] = ( uint8_t )( offset >> 8 );
    cbuffer[5] = ( uint8_t )( offset >> 0 );

    for( uint8_t i = 0; i < length; i++ )
    {
        cdata[i * sizeof( uint32_t ) + 0] = ( uint8_t )( data[i] >> 24 );
        cdata[i * sizeof( uint32_t ) + 1] = ( uint8_t )( data[i] >> 16 );
        cdata[i * sizeof( uint32_t ) + 2] = ( uint8_t )( data[i] >> 8 );
        cdata[i * sizeof( uint32_t ) + 3] = ( uint8_t )( data[i] >> 0 );
    }

    lr1110_hal_write( radio, cbuffer, LR1110_BL_WRITE_FLASH_ENCRYPTED_CMD_LENGTH, cdata, length * sizeof( uint32_t ) );
}

void lr1110_bootloader_write_flash_encrypted_full( const void* radio, const uint32_t offset, const uint32_t* buffer,
                                                   const uint32_t length )
{
    uint32_t remaining_length = length;
    uint32_t local_offset     = offset;
    uint32_t loop             = 0;

    while( remaining_length != 0 )
    {
        lr1110_bootloader_write_flash_encrypted( radio, local_offset, buffer + loop * 64, min( remaining_length, 64 ) );

        local_offset += LR1110_FLASH_DATA_UINT8_MAX;
        remaining_length = ( remaining_length < 64 ) ? 0 : ( remaining_length - 64 );

        loop++;
    }
}

void lr1110_bootloader_get_hash( const void* radio, lr1110_bootloader_hash_t hash )
{
    uint8_t cbuffer[LR1110_BL_GET_HASH_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_BL_GET_HASH_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_BL_GET_HASH_OC >> 0 );

    lr1110_hal_read( radio, cbuffer, LR1110_BL_GET_HASH_CMD_LENGTH, hash, LR1110_BL_HASH_LENGTH );
}

void lr1110_bootloader_reboot( const void* radio, const bool stay_in_bootloader )
{
    uint8_t cbuffer[LR1110_BL_REBOOT_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_BL_REBOOT_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_BL_REBOOT_OC >> 0 );

    cbuffer[2] = ( stay_in_bootloader == true ) ? 0x03 : 0x00;

    lr1110_hal_write( radio, cbuffer, LR1110_BL_REBOOT_CMD_LENGTH, 0, 0 );
}

void lr1110_bootloader_read_pin( const void* radio, lr1110_bootloader_pin_t pin )
{
    uint8_t cbuffer[LR1110_BL_GET_PIN_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_BL_GET_PIN_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_BL_GET_PIN_OC >> 0 );

    lr1110_hal_read( radio, cbuffer, LR1110_BL_GET_PIN_CMD_LENGTH, pin, LR1110_BL_PIN_LENGTH );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
