/*!
 * \file      lr1110_regmem.c
 *
 * \brief     Register/memory driver implementation for LR1110
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

#include "lr1110_regmem.h"
#include "lr1110_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR1110_REGMEM_CLEAR_RXBUFFER_CMD_LENGTH 2
#define LR1110_REGMEM_WRITE_AUXREG32_CMD_LENGTH ( 2 + 4 )
#define LR1110_REGMEM_READ_AUXREG32_CMD_LENGTH ( 2 + 4 + 1 )
#define LR1110_REGMEM_WRITE_REGMEM32_CMD_LENGTH ( 2 + 4 )
#define LR1110_REGMEM_READ_REGMEM32_CMD_LENGTH ( 2 + 4 + 1 )
#define LR1110_REGMEM_WRITE_MEM8_CMD_LENGTH ( 2 + 4 )
#define LR1110_REGMEM_READ_MEM8_CMD_LENGTH ( 2 + 4 + 1 )
#define LR1110_REGMEM_WRITE_BUFFER8_CMD_LENGTH ( 2 )
#define LR1110_REGMEM_READ_BUFFER8_CMD_LENGTH ( 2 + 2 )
#define LR1110_REGMEM_WRITE_REGMEM32_MASK_CMD_LENGTH ( 2 + 4 + 4 + 4 )

#define LR1110_REGMEM_BUFFER_SIZE_MAX ( 256 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

enum
{
    LR1110_REGMEM_NO_OP                  = 0x0000,
    LR1110_REGMEM_WRITE_AUXREG32_MASK_OC = 0x0102,
    LR1110_REGMEM_WRITE_AUXREG32_OC      = 0x0103,
    LR1110_REGMEM_READ_AUXREG32_OC       = 0x0104,
    LR1110_REGMEM_WRITE_REGMEM32_OC      = 0x0105,
    LR1110_REGMEM_READ_REGMEM32_OC       = 0x0106,
    LR1110_REGMEM_WRITE_MEM8_OC          = 0x0107,
    LR1110_REGMEM_READ_MEM8_OC           = 0x0108,
    LR1110_REGMEM_WRITE_BUFFER8_OC       = 0x0109,
    LR1110_REGMEM_READ_BUFFER8_OC        = 0x010A,
    LR1110_REGMEM_CLEAR_RXBUFFER_OC      = 0x010B,
    LR1110_REGMEM_WRITE_REGMEM32_MASK_OC = 0x010C,
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

void lr1110_regmem_write_auxreg32( const void* radio, const uint32_t address, const uint32_t* buffer,
                                   const uint8_t length )
{
    uint8_t cbuffer[LR1110_REGMEM_WRITE_AUXREG32_CMD_LENGTH];
    uint8_t cdata[LR1110_REGMEM_BUFFER_SIZE_MAX];

    cbuffer[0] = ( uint8_t )( LR1110_REGMEM_WRITE_AUXREG32_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_REGMEM_WRITE_AUXREG32_OC >> 0 );

    cbuffer[2] = ( uint8_t )( address >> 24 );
    cbuffer[3] = ( uint8_t )( address >> 16 );
    cbuffer[4] = ( uint8_t )( address >> 8 );
    cbuffer[5] = ( uint8_t )( address >> 0 );

    for( uint16_t index = 0; index < length; index++ )
    {
        uint8_t* cdata_local = &cdata[index * sizeof( uint32_t )];

        cdata_local[0] = ( uint8_t )( buffer[index] >> 24 );
        cdata_local[1] = ( uint8_t )( buffer[index] >> 16 );
        cdata_local[2] = ( uint8_t )( buffer[index] >> 8 );
        cdata_local[3] = ( uint8_t )( buffer[index] >> 0 );
    }

    lr1110_hal_write( radio, cbuffer, LR1110_REGMEM_WRITE_AUXREG32_CMD_LENGTH, cdata, length * sizeof( uint32_t ) );
}

void lr1110_regmem_read_auxreg32( const void* radio, const uint32_t address, uint32_t* buffer, const uint8_t length )
{
    uint8_t cbuffer[LR1110_REGMEM_READ_AUXREG32_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_REGMEM_READ_AUXREG32_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_REGMEM_READ_AUXREG32_OC >> 0 );

    cbuffer[2] = ( uint8_t )( address >> 24 );
    cbuffer[3] = ( uint8_t )( address >> 16 );
    cbuffer[4] = ( uint8_t )( address >> 8 );
    cbuffer[5] = ( uint8_t )( address >> 0 );

    cbuffer[6] = length;

    lr1110_hal_read( radio, cbuffer, LR1110_REGMEM_READ_AUXREG32_CMD_LENGTH, ( uint8_t* ) buffer,
                     length * sizeof( uint32_t ) );

    for( uint8_t index = 0; index < length; index++ )
    {
        uint8_t* buffer_local = ( uint8_t* ) &buffer[index];

        buffer[index] = ( ( uint32_t ) buffer_local[0] << 24 ) + ( ( uint32_t ) buffer_local[1] << 16 ) +
                        ( ( uint32_t ) buffer_local[2] << 8 ) + ( ( uint32_t ) buffer_local[3] << 0 );
    }
}

void lr1110_regmem_write_regmem32( const void* radio, const uint32_t address, const uint32_t* buffer,
                                   const uint8_t length )
{
    uint8_t cbuffer[LR1110_REGMEM_WRITE_REGMEM32_CMD_LENGTH];
    uint8_t cdata[LR1110_REGMEM_BUFFER_SIZE_MAX];

    cbuffer[0] = ( uint8_t )( LR1110_REGMEM_WRITE_REGMEM32_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_REGMEM_WRITE_REGMEM32_OC >> 0 );

    cbuffer[2] = ( uint8_t )( address >> 24 );
    cbuffer[3] = ( uint8_t )( address >> 16 );
    cbuffer[4] = ( uint8_t )( address >> 8 );
    cbuffer[5] = ( uint8_t )( address >> 0 );

    for( uint16_t index = 0; index < length; index++ )
    {
        uint8_t* cdata_local = &cdata[index * sizeof( uint32_t )];

        cdata_local[0] = ( uint8_t )( buffer[index] >> 24 );
        cdata_local[1] = ( uint8_t )( buffer[index] >> 16 );
        cdata_local[2] = ( uint8_t )( buffer[index] >> 8 );
        cdata_local[3] = ( uint8_t )( buffer[index] >> 0 );
    }

    lr1110_hal_write( radio, cbuffer, LR1110_REGMEM_WRITE_REGMEM32_CMD_LENGTH, cdata, length * sizeof( uint32_t ) );
}

void lr1110_regmem_read_regmem32( const void* radio, const uint32_t address, uint32_t* buffer, const uint8_t length )
{
    uint8_t cbuffer[LR1110_REGMEM_READ_REGMEM32_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_REGMEM_READ_REGMEM32_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_REGMEM_READ_REGMEM32_OC >> 0 );

    cbuffer[2] = ( uint8_t )( address >> 24 );
    cbuffer[3] = ( uint8_t )( address >> 16 );
    cbuffer[4] = ( uint8_t )( address >> 8 );
    cbuffer[5] = ( uint8_t )( address >> 0 );

    cbuffer[6] = length;

    lr1110_hal_read( radio, cbuffer, LR1110_REGMEM_READ_REGMEM32_CMD_LENGTH, ( uint8_t* ) buffer,
                     length * sizeof( uint32_t ) );

    for( uint8_t index = 0; index < length; index++ )
    {
        uint8_t* buffer_local = ( uint8_t* ) &buffer[index];

        buffer[index] = ( ( uint32_t ) buffer_local[0] << 24 ) + ( ( uint32_t ) buffer_local[1] << 16 ) +
                        ( ( uint32_t ) buffer_local[2] << 8 ) + ( ( uint32_t ) buffer_local[3] << 0 );
    }
}

void lr1110_regmem_write_mem8( const void* radio, const uint32_t address, const uint8_t* buffer, const uint8_t length )
{
    uint8_t cbuffer[LR1110_REGMEM_WRITE_MEM8_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_REGMEM_WRITE_MEM8_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_REGMEM_WRITE_MEM8_OC >> 0 );

    cbuffer[2] = ( uint8_t )( address >> 24 );
    cbuffer[3] = ( uint8_t )( address >> 16 );
    cbuffer[4] = ( uint8_t )( address >> 8 );
    cbuffer[5] = ( uint8_t )( address >> 0 );

    lr1110_hal_write( radio, cbuffer, LR1110_REGMEM_WRITE_MEM8_CMD_LENGTH, buffer, length );
}

void lr1110_regmem_read_mem8( const void* radio, const uint32_t address, uint8_t* buffer, const uint8_t length )
{
    uint8_t cbuffer[LR1110_REGMEM_READ_MEM8_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_REGMEM_READ_MEM8_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_REGMEM_READ_MEM8_OC >> 0 );

    cbuffer[2] = ( uint8_t )( address >> 24 );
    cbuffer[3] = ( uint8_t )( address >> 16 );
    cbuffer[4] = ( uint8_t )( address >> 8 );
    cbuffer[5] = ( uint8_t )( address >> 0 );

    cbuffer[6] = length;

    lr1110_hal_read( radio, cbuffer, LR1110_REGMEM_READ_MEM8_CMD_LENGTH, buffer, length );
}

void lr1110_regmem_write_buffer8( const void* radio, const uint8_t* buffer, const uint8_t length )
{
    uint8_t cbuffer[LR1110_REGMEM_WRITE_BUFFER8_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_REGMEM_WRITE_BUFFER8_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_REGMEM_WRITE_BUFFER8_OC >> 0 );

    lr1110_hal_write( radio, cbuffer, LR1110_REGMEM_WRITE_BUFFER8_CMD_LENGTH, buffer, length );
}

void lr1110_regmem_read_buffer8( const void* radio, uint8_t* buffer, const uint8_t offset, const uint8_t length )
{
    uint8_t cbuffer[LR1110_REGMEM_READ_BUFFER8_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_REGMEM_READ_BUFFER8_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_REGMEM_READ_BUFFER8_OC >> 0 );

    cbuffer[2] = offset;
    cbuffer[3] = length;

    lr1110_hal_read( radio, cbuffer, LR1110_REGMEM_READ_BUFFER8_CMD_LENGTH, buffer, length );
}

void lr1110_regmem_clear_rxbuffer( const void* radio )
{
    uint8_t cbuffer[LR1110_REGMEM_CLEAR_RXBUFFER_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_REGMEM_CLEAR_RXBUFFER_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_REGMEM_CLEAR_RXBUFFER_OC >> 0 );

    lr1110_hal_write( radio, cbuffer, LR1110_REGMEM_CLEAR_RXBUFFER_CMD_LENGTH, 0, 0 );
}

void lr1110_regmem_write_regmem32_mask( const void* radio, const uint32_t address, const uint32_t mask,
                                        const uint32_t data )
{
    uint8_t cbuffer[LR1110_REGMEM_WRITE_REGMEM32_MASK_CMD_LENGTH];

    cbuffer[0] = ( uint8_t )( LR1110_REGMEM_WRITE_REGMEM32_MASK_OC >> 8 );
    cbuffer[1] = ( uint8_t )( LR1110_REGMEM_WRITE_REGMEM32_MASK_OC >> 0 );

    cbuffer[2] = ( uint8_t )( address >> 24 );
    cbuffer[3] = ( uint8_t )( address >> 16 );
    cbuffer[4] = ( uint8_t )( address >> 8 );
    cbuffer[5] = ( uint8_t )( address >> 0 );

    cbuffer[6] = ( uint8_t )( mask >> 24 );
    cbuffer[7] = ( uint8_t )( mask >> 16 );
    cbuffer[8] = ( uint8_t )( mask >> 8 );
    cbuffer[9] = ( uint8_t )( mask >> 0 );

    cbuffer[10] = ( uint8_t )( data >> 24 );
    cbuffer[11] = ( uint8_t )( data >> 16 );
    cbuffer[12] = ( uint8_t )( data >> 8 );
    cbuffer[13] = ( uint8_t )( data >> 0 );

    lr1110_hal_write( radio, cbuffer, LR1110_REGMEM_WRITE_REGMEM32_MASK_CMD_LENGTH, 0, 0 );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
