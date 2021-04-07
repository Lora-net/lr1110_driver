/*!
 * @file      lr1110_hal.h
 *
 * @brief     Hardware Abstraction Layer (HAL) interface for LR1110
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
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LR1110_HAL_H
#define LR1110_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*!
 * @brief LR1110 HAL status
 */
typedef enum lr1110_hal_status_e
{
    LR1110_HAL_STATUS_OK    = 0,
    LR1110_HAL_STATUS_ERROR = 3,
} lr1110_hal_status_t;

/*
 * ============================================================================
 * API definitions to be implemented by the user
 * ============================================================================
 */

/*!
 * @brief Radio data transfer - write
 *
 * @remark Must be implemented by the upper layer
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer to be transmitted
 * @param [in] data_length      Buffer size to be transmitted
 *
 * @returns Operation status
 */
lr1110_hal_status_t lr1110_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length );

/*!
 * @brief Radio data transfer - read
 *
 * @remark Must be implemented by the upper layer
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [out] data            Pointer to the buffer to be received
 * @param [in] data_length      Buffer size to be received
 *
 * @returns Operation status
 */
lr1110_hal_status_t lr1110_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length );

/*!
 * @brief  Radio data transfer - write & read in single operation
 *
 * @remark Must be implemented by the upper layer
 * @remark Only required by lr1110_system_get_status command
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [out] data            Pointer to the buffer to be received
 * @param [in] data_length      Buffer size to be received
 *
 * @returns Operation status
 */
lr1110_hal_status_t lr1110_hal_write_read( const void* context, const uint8_t* command, uint8_t* data,
                                           const uint16_t data_length );

/*!
 * @brief Reset the radio
 *
 * @remark Must be implemented by the upper layer
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
lr1110_hal_status_t lr1110_hal_reset( const void* context );

/*!
 * @brief Wake the radio up.
 *
 * @remark Must be implemented by the upper layer
 *
 * @param [in] context Radio implementation parameters

 * @returns Operation status
 */
lr1110_hal_status_t lr1110_hal_wakeup( const void* context );

/*!
 * @brief Return the computed CRC
 *
 * @param [in] initial_value initial value of the CRC
 * @param [in] buffer Buffer containing data used to compute the CRC
 * @param [in] length Length of buffer
 *
 * @returns CRC value
 */
inline static uint8_t lr1110_hal_compute_crc( const uint8_t initial_value, const uint8_t* buffer, uint16_t length )
{
    uint8_t crc = initial_value;

    for( uint16_t i = 0; i < length; i++ )
    {
        uint8_t extract = buffer[i];
        uint8_t sum;

        for( uint8_t j = 8; j > 0; j-- )
        {
            sum = ( crc ^ extract ) & 0x01;
            crc >>= 1;

            if( sum != 0 )
            {
                crc ^= 0x65;
            }

            extract >>= 1;
        }
    }

    return crc;
}

#ifdef __cplusplus
}
#endif

#endif  // LR1110_HAL_H
