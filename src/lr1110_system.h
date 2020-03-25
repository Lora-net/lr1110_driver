/*!
 * \file      lr1110_system.h
 *
 * \brief     System driver definition for LR1110
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

#ifndef __LR1110_SYSTEM_H__
#define __LR1110_SYSTEM_H__

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr1110_system_types.h"

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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief Reset the radio
 *
 * \param [in] radio Radio abstraction
 *
 * \param [in] radio Pointer to the structure holding the radio context
 */
void lr1110_system_reset( const void* radio );

/*!
 * \brief Return the version of the system (hardware and software)
 *
 * \param [in] radio Radio abstraction
 *
 * \param [out] version Pointer to the structure holding the system version
 */
void lr1110_system_get_status( const void* radio, lr1110_system_stat1_t* stat1, lr1110_system_stat2_t* stat2,
                               uint32_t* irq_status );

/*!
 * \brief Return the version of the system (hardware and software)
 *
 * \param [in] radio Radio abstraction
 *
 * \param [out] version Pointer to the structure holding the system version
 */
void lr1110_system_get_version( const void* radio, lr1110_system_version_t* version );

/*!
 * \brief Return the system errors
 *
 * Errors may be fixed following:
 * - calibration error can be fixed by attempting another RC calibration;
 * - XOsc related errors may be due to hardware problems, can be fixed by
 * reset;
 * - PLL lock related errors can be due to not-locked PLL, or by attempting to
 * use an out-of-band frequency, can be fix by executing a PLL calibration, or
 * by using other frequencies.
 *
 * \param [in] radio Radio abstraction
 *
 * \param [out] errors Pointer to a value holding error flags
 *
 * \see lr1110_system_calibrate, lr1110_system_calibrate_image,
 * lr1110_system_clear_errors
 */
void lr1110_system_get_errors( const void* radio, uint16_t* errors );

/*!
 * \brief Clear all error flags pending.
 *
 * This function cannot be used to clear flags individually.
 *
 * \param [in] radio Radio abstraction
 *
 * \see lr1110_system_get_errors
 */
void lr1110_system_clear_errors( const void* radio );

/*!
 * \brief lr1110_system_calibrate the requested blocks
 *
 * This function can be called in any mode of the chip.
 *
 * The chip will return to standby RC mode on exit. Potential calibration
 * issues can be read out with lr1110_system_get_errors command.
 *
 * \param [in] radio Radio abstraction
 *
 * \param [in] calib_params Structure holding the reference to blocks to be
 * calibrated
 *
 * \see lr1110_system_get_errors
 */
void lr1110_system_calibrate( const void* radio, const uint8_t calib_params );

/*!
 * \brief Configure the regulator mode to be used in specific modes
 *
 * This function shall only be called in standby RC mode.
 *
 * The reg_mode parameter defines if the DC-DC converter is switched on in the
 * following modes: STANDBY XOSC, FS, RX, TX and RX_CAPTURE.
 *
 * \param [in] radio Radio abstraction
 *
 * \param [in] reg_mode Regulator mode configuration
 */
void lr1110_system_set_regmode( const void* radio, const lr1110_regmodes_t reg_mode );

/*!
 * \brief Launch an image calibration at frequencies given in parameters
 *
 * This function can be called in any mode of the chip.
 *
 * The chip will return to standby RC mode on exit. Potential calibration
 * issues can be read out with lr1110_system_get_errors command.
 *
 * The frequencies given in parameters are defined in 4MHz step (Eg. 900MHz
 * corresponds to 0xE1). If freq1 = freq2, only one calibration is performed.
 *
 * \param [in] radio Radio abstraction
 *
 * \param [in] freq1 First frequency used to perform image calibration
 *
 * \param [in] freq2 Second frequency used to perform image calibration
 *
 * \see lr1110_system_get_errors
 */
void lr1110_system_calibrate_image( const void* radio, const uint8_t freq1, const uint8_t freq2 );

/*!
 * \brief Set the RF switch configurations for each RF setup
 *
 * This function shall only be called in standby RC mode.
 *
 * By default, no DIO is used to control a RF switch. All DIOs are set in
 * High-Z mode.
 *
 * \param [in] radio Radio abstraction
 *
 * \param [in] rfSwitch Pointer to a structure that holds the switches
 * configuration
 */
void lr1110_system_set_dio_as_rf_switch( const void*                            radio,
                                         const lr1110_system_rfswitch_config_t* rf_switch_configuration );

/*!
 * \brief Set which interrupt signals are redirected to the dedicated DIO pin
 *
 * By default, no interrupt signal is redirected.
 *
 * The dedicated DIO pin will remain asserted until all redirected interrupt
 * signals are cleared with a call to lr1110_system_clear_irq.
 *
 * \param [in] radio Radio abstraction
 *
 * \param [in] irqs_to_enable_dio1 Variable that holds the interrupt mask for
 * dio1
 *
 * \param [in] irqs_to_enable_dio2 Variable that holds the interrupt mask for
 * dio2
 *
 * \see lr1110_system_clear_irq
 */
void lr1110_system_set_dio_irq_params( const void* radio, const uint32_t irqs_to_enable_dio1,
                                       const uint32_t irqs_to_enable_dio2 );

/*!
 * \brief Clear requested bits in the internal pending interrupt register
 *
 * \param [in] radio Radio abstraction
 *
 * \param [in] irqs_to_clear Variable that holds the interrupts to be cleared
 */
void lr1110_system_clear_irq( const void* radio, const uint32_t irqs_to_clear );

/*!
 * \brief Defines which clock is used as Low Frequency (LF) clock
 *
 * \param [in] radio Radio abstraction
 *
 * \param [in] lfclock_config Low frequency clock configuration
 *
 * \param [in] wait_for_32k_ready
 *
 * \see lr1110_system_calibrate, lr1110_system_calibrate_image
 */
void lr1110_system_config_lfclk( const void* radio, const lr1110_system_lfclk_config_t lfclock_config,
                                 const bool wait_for_32k_ready );

/*!
 * \brief Enable and configure TCXO supply voltage and detection timeout
 *
 * This function shall only be called in standby RC mode.
 *
 * The timeout parameter is the maximum time the firmware waits for the TCXO
 * to be ready. The timeout duration is given by:
 * \f$ timeout\_duration\_us = timeout \times 30.52 \f$
 *
 * The TCXO mode can be disabled by setting timeout parameter to 0.
 *
 * \param [in] radio Radio abstraction
 *
 * \param [in] tune Supply voltage value
 *
 * \param [in] timeout
 *
 * \see lr1110_system_calibrate, lr1110_system_calibrate_image
 */
void lr1110_system_set_tcxo_mode( const void* radio, const lr1110_system_tcxo_supply_voltage_t tune,
                                  const uint32_t timeout );

/*!
 * \brief Software reset of the chip.
 *
 * This function should be used to reboot the chip in a specified mode.
 * Rebooting in flash mode presumes that the content in flash memory is not
 * corrupted (i.e. the integrity check performed by the bootloader before
 * executing the first instruction in flash is OK).
 *
 * \param [in] radio Radio abstraction
 *
 * \param [in] stay_in_bootloader Selector to stay in bootloader or execute
 * flash code after reboot. If true, the bootloader will not execute the flash
 * code but activate SPI interface to allow firmware upgrade
 */
void lr1110_system_reboot( const void* radio, const bool stay_in_bootloader );

/*!
 * \brief Returns the value of Vbat
 *
 * Vbat value (in V) is a function of Vana (typ. 1.35V) using the following
 * formula: \f$ Vbat_{V} = (5 \times \frac{Vbat}{255} - 1) \times Vana \f$
 *
 * \param [in] radio Radio abstraction
 *
 * \param [out] vbat A pointer to the Vbat value
 */
void lr1110_system_get_vbat( const void* radio, uint8_t* vbat );

/*!
 * \brief Returns the value of Temp
 *
 * The temperature (in °C) is a function of Vana (typ. 1.35V), Vbe25 (Vbe
 * voltage @ 25°C, typ. 0.7295V) and VbeSlope (typ. -1.7mV/°C) using the
 * following formula: \f$ Temperature_{°C} = (\frac{Temp(10:0)}{2047} \times
 * Vana - Vbe25) \times \frac{1000}{VbeSlope} + 25 \f$
 *
 * \param [in] radio Radio abstraction
 *
 * \param [out] temp A pointer to the Temp value
 */
void lr1110_system_get_temp( const void* radio, uint16_t* temp );

/*!
 * \brief Set the device into Sleep or Deep Sleep Mode
 *
 * The sleep_config parameter defines in which sleep mode the device is put and
 * if it wakes up after a given time on the RTC event.
 *
 * The sleep_time parameter is taken into account only when RtcTimeout = 1. It
 * sets the sleep time in number of clock cycles: \f$ sleep\_time\_ms =
 * sleep_time \times \frac{1}{32.768} \f$
 *
 * \param [in] radio Radio abstraction
 *
 * \param [in] sleep_config Sleep mode configuration
 *
 * \param [in] sleep_time Value of the RTC timeout (if RtcTimeout = 1)
 *
 * \see lr1110_system_set_standby, lr1110_system_set_fs
 */
void lr1110_system_set_sleep( const void* radio, const lr1110_system_sleep_config_t sleep_config,
                              const uint32_t sleep_time );

/*!
 * \brief Set the device into the requested Standby mode
 *
 * \param [in] radio Radio abstraction
 *
 * \param [in] standby_config Stand by mode configuration (RC or XOSC)
 *
 * \see lr1110_system_set_sleep, lr1110_system_set_fs
 */
void lr1110_system_set_standby( const void* radio, const lr1110_system_standby_config_t standby_config );

/*!
 * \brief Set the device into Frequency Synthesis (FS) mode
 *
 * \param [in] radio Radio abstraction
 *
 * \see lr1110_system_set_standby, lr1110_system_set_sleep
 */
void lr1110_system_set_fs( const void* radio );

/*!
 * \brief Erase an info page
 *
 * \param [in] radio Radio abstraction
 *
 * \param [in] info_page_id Info page to be erased
 *
 * \see lr1110_system_write_infopage, lr1110_system_read_infopage
 */
void lr1110_system_erase_infopage( const void* radio, const lr1110_system_infopage_id_t info_page_id );

/*!
 * \brief Write data in an info page
 *
 * It is possibe to cross from page 0 to 1 if (address + length >= 512)
 *
 * \param [in] radio Radio abstraction
 *
 * \param [in] info_page_id Info page where data are written
 *
 * \param [in] address Address within the info page (aligned on 32-bit data)
 *
 * \param [in] data Pointer to the data to write (data buffer shall be - at
 * least - length words long)
 *
 * \param [in] length Number of 32-bit data to write (maximum value is 64)
 *
 * \see lr1110_system_erase_infopage, lr1110_system_read_infopage
 */
void lr1110_system_write_infopage( const void* radio, const lr1110_system_infopage_id_t info_page_id,
                                   const uint16_t address, const uint32_t* data, const uint8_t length );

/*!
 * \brief Read data from an info page
 *
 * It is possible to cross from page 0 to 1 if (address + length >= 512)
 *
 * \param [in] radio Radio abstraction
 *
 * \param [in] info_page_id Info page where data are read
 *
 * \param [in] address Address within the info page (aligned on 32-bit data)
 *
 * \param [out] data Pointer to the data to read (data buffer shall be - at
 * least - length words long)
 *
 * \param [in] length Number of 32-bit data to read (maximum value is 64)
 *
 * \see lr1110_system_erase_infopage, lr1110_system_write_infopage
 */
void lr1110_system_read_infopage( const void* radio, const lr1110_system_infopage_id_t info_page_id,
                                  const uint16_t address, uint32_t* data, const uint8_t length );

/*!
 * \brief Read and return the Unique Identifier of the LR1110
 *
 * \param [in] radio Radio abstraction
 *
 * \param [out] unique_identifier The buffer to be filled with the Unique
 * Identifier of the LR1110. It is up to the application to ensure
 * unique_identifier is long enough to hold the unique identifier
 *
 * \see LR1110_SYSTEM_UID_LENGTH
 */
void lr1110_system_read_uid( const void* radio, lr1110_system_uid_t unique_identifier );

/*!
 * \brief Read and return the Join EUI of the LR1110
 *
 * \param [in] radio Radio abstraction
 *
 * \param [out] join_eui The buffer to be filled with Join EUI of the LR1110.
 * It is up to the application to ensure join_eui is long enough to
 * hold the join EUI
 *
 * \see LR1110_SYSTEM_JOIN_EUI_LENGTH
 */
void lr1110_system_read_join_eui( const void* radio, lr1110_system_join_eui_t join_eui );

/*!
 * \brief Read and return the PIN of the LR1110
 *
 * \param [in] radio Radio abstraction
 *
 * \param [out] pin The buffer to be filled with PIN of the LR1110.
 * It is up to the application to ensure pin is long enough to
 * hold the PIN
 *
 * \see LR1110_SYSTEM_PIN_LENGTH
 */
void lr1110_system_read_pin( const void* radio, lr1110_system_pin_t pin );

/*!
 * \brief Read and return a 32-bit random number
 *
 * \remark Radio operating mode must be set into standby.
 *
 * \param [in] radio Radio abstraction
 *
 * \param [out] random_number 32-bit random number
 */
void lr1110_system_get_random_number( const void* radio, uint32_t* random_number );

#ifdef __cplusplus
}
#endif

#endif  // __LR1110_SYSTEM_H__

/* --- EOF ------------------------------------------------------------------ */
