# LR1110 driver

## [v2.0.0] 2020-04-27

### Added

* [all] All functions declared in .h files now return a status.
* [common] `lr1110_status_t` type definition
* [bootloader] `lr1110_bootloader_fill_cbuffer_opcode_offset_flash` static function
* [bootloader] `lr1110_bootloader_fill_cdata_flash` static function
* [bootloader] `lr1110_bootloader_fill_cbuffer_cdata_flash` static function
* [regmem] `lr1110_regmem_fill_cbuffer_opcode_address` static function
* [regmem] `lr1110_regmem_fill_cbuffer_opcode_address_length` static function
* [regmem] `lr1110_regmem_fill_cdata` static function
* [regmem] `lr1110_regmem_fill_cbuffer_cdata_opcode_address_data` static function
* [regmem] `lr1110_regmem_fill_out_buffer_from_raw_buffer` static function
* [system] `LR1110_SYSTEM_VERSION_LENGTH` constant
* [system] `lr1110_system_cal_mask_t` type definition
* [system] `lr1110_system_irq_mask_t` type definition
* [system] `lr1110_system_get_and_clear_irq_status` function
* [system] `lr1110_system_get_irq_status` function
* [crypto] `lr1110_crypto_fill_cbuffer_opcode_key_data` static function
* [GNSS] `lr1110_gnss_uint8_to_uint32` static function

### Changed

* [bootloader] `lr1110_bootloader_version_t` has now 3 fields: hardware, type, firmware
* [bootloader] `lr1110_bootloader_get_version` fills the updated `lr1110_bootloader_version_t` structure
* [system] `LR1110_SYSTEM_IRQ_NONE_MASK` is renamed `LR1110_SYSTEM_IRQ_NONE`
* [system] `LR1110_SYSTEM_IRQ_TXDONE_MASK` is renamed `LR1110_SYSTEM_IRQ_TX_DONE`
* [system] `LR1110_SYSTEM_IRQ_RXDONE_MASK` is renamed `LR1110_SYSTEM_IRQ_RX_DONE`
* [system] `LR1110_SYSTEM_IRQ_PREAMBLEDETECTED_MASK` is renamed `LR1110_SYSTEM_IRQ_PREAMBLE_DETECTED`
* [system] `LR1110_SYSTEM_IRQ_SYNCWORD_HEADERVALID_MASK` is renamed `LR1110_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID`
* [system] `LR1110_SYSTEM_IRQ_HEADERERR_MASK` is renamed `LR1110_SYSTEM_IRQ_HEADER_ERROR`
* [system] `LR1110_SYSTEM_IRQ_CRCERR_MASK` is renamed `LR1110_SYSTEM_IRQ_CRC_ERROR`
* [system] `LR1110_SYSTEM_IRQ_CADDONE_MASK` is renamed `LR1110_SYSTEM_IRQ_CAD_DONE`
* [system] `LR1110_SYSTEM_IRQ_CADDETECTED_MASK` is renamed `LR1110_SYSTEM_IRQ_CAD_DETECTED`
* [system] `LR1110_SYSTEM_IRQ_TIMEOUT_MASK` is renamed `LR1110_SYSTEM_IRQ_TIMEOUT`
* [system] `LR1110_SYSTEM_IRQ_GNSSSCANDONE_MASK` is renamed `LR1110_SYSTEM_IRQ_GNSS_SCAN_DONE`
* [system] `LR1110_SYSTEM_IRQ_WIFISCANDONE_MASK` is renamed `LR1110_SYSTEM_IRQ_WIFI_SCAN_DONE`
* [system] `LR1110_SYSTEM_IRQ_EOL_MASK` is renamed `LR1110_SYSTEM_IRQ_EOL`
* [system] `LR1110_SYSTEM_IRQ_CMDERR_MASK` is renamed `LR1110_SYSTEM_IRQ_CMD_ERROR`
* [system] `LR1110_SYSTEM_IRQ_ERR_MASK` is renamed `LR1110_SYSTEM_IRQ_ERROR`
* [system] `LR1110_SYSTEM_IRQ_FSK_LENGTH_ERROR_MASK` is renamed `LR1110_SYSTEM_IRQ_FSK_LEN_ERROR`
* [system] `LR1110_SYSTEM_IRQ_FSK_ADDRESS_ERROR_MASK` is renamed `LR1110_SYSTEM_IRQ_FSK_ADDR_ERROR`
* [system] `LR1110_SYSTEM_CALIBRATE_*_MASK` are renamed `LR1110_SYSTEM_CALIB_*_MASK`
* [system] `lr1110_system_chip_mode_t` is renamed `lr1110_system_chip_modes_t`
* [system] In `lr1110_system_chip_modes_t`, `LR1110_SYSTEM_CHIP_MODE_RC` is renamed `LR1110_SYSTEM_CHIP_MODE_STBY_RC`
* [system] In `lr1110_system_chip_modes_t`, `LR1110_SYSTEM_CHIP_MODE_XOSC` is renamed `LR1110_SYSTEM_CHIP_MODE_STBY_XOSC`
* [system] `lr1110_system_lfclk_config_t` is renamed `lr1110_system_lfclk_cfg_t`
* [system] `lr1110_regmodes_t` is renamed `lr1110_system_reg_mode_t`
* [system] `LR1110_SYSTEM_REGMODE_NO_DCDC` is renamed `LR1110_SYSTEM_REG_MODE_LDO`
* [system] `LR1110_SYSTEM_REGMODE_DCDC_CONVERTER` is renamed `LR1110_SYSTEM_REG_MODE_DCDC`
* [system] `lr1110_system_rfswitch_config_t` is renamed `lr1110_system_rfswitch_cfg_t`
* [system] `lr1110_system_standby_config_t` is renamed `lr1110_system_standby_cfg_t`
* [system] `LR1110_SYSTEM_STDBY_CONFIG_*` are renamed `LR1110_SYSTEM_STANDBY_CFG_*`
* [system] `LR1110_SYSTEM_TCXO_SUPPLY_VOLTAGE_*V` are renamed `LR1110_SYSTEM_TCXO_CTRL_*V`
* [system] `lr1110_system_sleep_config_t` is renamed `lr1110_system_sleep_cfg_t`
* [system] `lr1110_system_set_regmode` is renamed `lr1110_system_set_reg_mode`
* [system] `lr1110_system_config_lfclk` is renamed `lr1110_system_cfg_lfclk`
* [system] `lr1110_system_clear_irq` is renamed `lr1110_system_clear_irq_status`
* [crypto] `lr1110_crypto_derive_and_store_key` is renamed `lr1110_crypto_derive_key`
* [crypto] `LR1110_CRYPTO_DERIVE_AND_STORE_KEY_OC` is renamed `LR1110_CRYPTO_DERIVE_KEY_OC`
* [crypto] `LR1110_CRYPTO_DERIVE_AND_STORE_KEY_CMD_LENGTH` is renamed `LR1110_CRYPTO_DERIVE_KEY_CMD_LENGTH`
* [radio] `lr1110_radio_rx_tx_fallback_mode_t` is renamed `lr1110_radio_fallback_modes_t`
* [radio] `LR1110_RADIO_RX_TX_FALLBACK_MODE_*` are renamed `LR1110_RADIO_FALLBACK_*`
* [radio] `LR1110_RADIO_RAMP_TIME_*` are renamed `LR1110_RADIO_RAMP_*`
* [radio] `LR1110_RADIO_LORA_BW*` are renamed `LR1110_RADIO_LORA_BW_*`
* [radio] `LR1110_RADIO_LORA_CRXY_LI` are renamed `LR1110_RADIO_LORA_CR_LI_X_Y`
* [radio] `LR1110_RADIO_LORA_CRXY` are renamed `LR1110_RADIO_LORA_CR_X_Y`
* [radio] `LR1110_RADIO_MODE_STANDBY*` are renamed `LR1110_RADIO_MODE_STANDBY_*`
* [radio] `LR1110_RADIO_GFSK_CRC_XBYTE` are renamed `LR1110_RADIO_GFSK_CRC_X_BYTE`
* [radio] `LR1110_RADIO_GFSK_CRC_XBYTES` are renamed `LR1110_RADIO_GFSK_CRC_X_BYTES`
* [radio] `LR1110_RADIO_GFSK_CRC_XBYTE_INV` are renamed `LR1110_RADIO_GFSK_CRC_X_BYTE_INV`
* [radio] `LR1110_RADIO_GFSK_CRC_XBYTES_INV` are renamed `LR1110_RADIO_GFSK_CRC_X_BYTES_INV`
* [radio] `LR1110_RADIO_GFSK_DCFREE_*` are renamed `LR1110_RADIO_GFSK_DC_FREE_*`
* [radio] `lr1110_radio_gfsk_header_type_t` is renamed `lr1110_radio_gfsk_pkt_len_modes_t`
* [radio] `LR1110_RADIO_GFSK_HEADER_TYPE_IMPLICIT` is renamed `LR1110_RADIO_GFSK_PKT_FIX_LEN`
* [radio] `LR1110_RADIO_GFSK_HEADER_TYPE_EXPLICIT` is renamed `LR1110_RADIO_GFSK_PKT_VAR_LEN`
* [radio] `lr1110_radio_gfsk_preamble_detect_length_t` is renamed `lr1110_radio_gfsk_preamble_detector_t`
* [radio] `LR1110_RADIO_GFSK_PREAMBLE_DETECTOR_LENGTH_*` are renamed `LR1110_RADIO_GFSK_PREAMBLE_DETECTOR_*`
* [radio] `lr1110_radio_lora_header_type_t` is renamed `lr1110_radio_lora_pkt_len_modes_t`
* [radio] `LR1110_RADIO_LORA_HEADER_*` are renamed `LR1110_RADIO_LORA_PKT_*`
* [radio] `lr1110_radio_packet_types_t` is renamed `lr1110_radio_pkt_type_t`
* [radio] `LR1110_RADIO_PACKET_*` are renamed `LR1110_RADIO_PKT_TYPE_*`
* [radio] `lr1110_radio_gfsk_rx_bw_t` is renamed `lr1110_radio_gfsk_bw_t`
* [radio] `LR1110_RADIO_GFSK_RX_BW_*` are renamed `LR1110_RADIO_GFSK_BW_*`
* [radio] `lr1110_radio_pulse_shape_t` is renamed `lr1110_radio_gfsk_pulse_shape_t`
* [radio] `LR1110_RADIO_PULSESHAPE_*` are renamed `LR1110_RADIO_GFSK_PULSE_SHAPE_*`
* [radio] In `lr1110_radio_cad_params_t`, `symbol_num` is renamed `cad_symb_nb`
* [radio] In `lr1110_radio_cad_params_t`, `det_peak` is renamed `cad_detect_peak`
* [radio] In `lr1110_radio_cad_params_t`, `det_min` is renamed `cad_detect_min`
* [radio] In `lr1110_radio_cad_params_t`, `exit_mode` is renamed `cad_exit_mode`
* [radio] In `lr1110_radio_cad_params_t`, `timeout` is renamed `cad_timeout`
* [radio] `lr1110_radio_packet_status_gfsk_t` is renamed `lr1110_radio_pkt_status_gfsk_t`
* [radio] In `lr1110_radio_pkt_status_gfsk_t`, `rx_length_in_bytes` is renamed `rx_len_in_bytes`
* [radio] `lr1110_radio_packet_status_lora_t` is renamed `lr1110_radio_pkt_status_lora_t`
* [radio] `lr1110_radio_rxbuffer_status_t` is renamed `lr1110_radio_rx_buffer_status_t`
* [radio] In `lr1110_radio_rx_buffer_status_t`, `rx_payload_length` is renamed `pld_len_in_bytes`
* [radio] In `lr1110_radio_rx_buffer_status_t`, `rx_start_buffer_pointer` is renamed `buffer_start_pointer`
* [radio] In `lr1110_radio_stats_gfsk_t`, `nb_packet_*` are renamed `nb_pkt_*`
* [radio] In `lr1110_radio_stats_lora_t`, `nb_packet_received` is renamed `nb_pkt_received`
* [radio] In `lr1110_radio_stats_lora_t`, `nb_packet_error_crc` is renamed `nb_pkt_error_crc`
* [radio] In `lr1110_radio_stats_lora_t`, `nb_packet_error_header` is renamed `nb_pkt_header_error`
* [radio] `lr1110_radio_modulation_param_*_t` are renamed `lr1110_radio_mod_params_*_t`
* [radio] In `lr1110_radio_mod_params_gfsk_t`, `bitrate` is renamed `br_in_bps`
* [radio] In `lr1110_radio_mod_params_gfsk_t`, `bandwidth` is renamed `bw_dsb_param`
* [radio] In `lr1110_radio_mod_params_gfsk_t`, `fdev` is renamed `fdev_in_hz`
* [radio] In `lr1110_radio_mod_params_lora_t`, `spreading_factor` is renamed `sf`
* [radio] In `lr1110_radio_mod_params_lora_t`, `coding_rate` is renamed `cr`
* [radio] In `lr1110_radio_mod_params_lora_t`, `ppm_offset` is renamed `ldro`
* [radio] `lr1110_radio_packet_param_*_t` are renamed `lr1110_radio_pkt_params_*_t`
* [radio] In `lr1110_radio_pkt_params_gfsk_t`, `preamble_length_tx_in_bit` is renamed `preamble_len_in_bits`
* [radio] In `lr1110_radio_pkt_params_gfsk_t`, `preamble_detect` is renamed `preamble_detector`
* [radio] In `lr1110_radio_pkt_params_gfsk_t`, `sync_word_length_in_bit` is renamed `sync_word_len_in_bits`
* [radio] In `lr1110_radio_pkt_params_gfsk_t`, `payload_length_in_byte` is renamed `pld_len_in_bytes`
* [radio] In `lr1110_radio_pkt_params_lora_t`, `preamble_length_in_symb` is renamed `preamble_len_in_symb`
* [radio] In `lr1110_radio_pkt_params_lora_t`, `payload_length_in_byte` is renamed `pld_len_in_bytes`
* [radio] `lr1110_radio_pa_config_t` are renamed `lr1110_radio_pa_cfg_t`
* [Wi-Fi] `lr1110_wifi_configure_hardware_debarker` is renamed `lr1110_wifi_cfg_hardware_debarker`

### Fixed

* [system] Enable c linkage for system-related functions

### Removed

* [system] `LR1110_SYSTEM_IRQ_FHSS_MASK` constant
* [system] `LR1110_SYSTEM_IRQ_INTERPACKET1_MASK` constant
* [system] `LR1110_SYSTEM_IRQ_INTERPACKET2_MASK` constant
* [system] `LR1110_SYSTEM_IRQ_RNGREQVLD_MASK` constant
* [system] `LR1110_SYSTEM_IRQ_RNGREQDISC_MASK` constant
* [system] `LR1110_SYSTEM_IRQ_RNGRESPDONE_MASK` constant
* [system] `LR1110_SYSTEM_IRQ_RNGEXCHVLD_MASK` constant
* [system] `LR1110_SYSTEM_IRQ_RNGTIMEOUT_MASK` constant

## [v1.0.0] 2020-03-18

### Added

* [all] First public release
