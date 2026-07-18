#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sx126x.h"

// Protocol definitions for LoRa over SDIO using ESP32 running modified ESP-HOSTED

#define LORA_PROTOCOL_VERSION_STRING_LENGTH 16
#define LORA_FREQUENCY_ERROR_HISTORY_LENGTH 32

typedef enum {
    LORA_PROTOCOL_TYPE_ACK                  = 0x00,
    LORA_PROTOCOL_TYPE_NACK                 = 0x01,
    LORA_PROTOCOL_TYPE_GET_MODE             = 0x02,
    LORA_PROTOCOL_TYPE_SET_MODE             = 0x03,
    LORA_PROTOCOL_TYPE_GET_CONFIG           = 0x04,
    LORA_PROTOCOL_TYPE_SET_CONFIG           = 0x05,
    LORA_PROTOCOL_TYPE_GET_STATUS           = 0x06,
    LORA_PROTOCOL_TYPE_PACKET_RX            = 0x07,
    LORA_PROTOCOL_TYPE_PACKET_TX            = 0x08,
    LORA_PROTOCOL_TYPE_GET_RSSI_INST        = 0x09,
    LORA_PROTOCOL_TYPE_GET_FREQUENCY_OFFSET = 0x0A,
    LORA_PROTOCOL_TYPE_SET_FREQUENCY_OFFSET = 0x0B,
} lora_protocol_packet_type_t;

typedef enum {
    LORA_PROTOCOL_MODE_UNKNOWN      = 0x00,
    LORA_PROTOCOL_MODE_STANDBY_RC   = 0x01,
    LORA_PROTOCOL_MODE_STANDBY_XOSC = 0x02,
    LORA_PROTOCOL_MODE_FS           = 0x03,
    LORA_PROTOCOL_MODE_TX           = 0x04,
    LORA_PROTOCOL_MODE_RX           = 0x05,
} lora_protocol_mode_t;

typedef enum {
    LORA_PROTOCOL_CHIP_SX1262 = 0x00,
    LORA_PROTOCOL_CHIP_SX1268 = 0x01,
} lora_protocol_chip_t;

typedef struct {
    lora_protocol_mode_t mode;
} __attribute__((packed)) lora_protocol_mode_params_t;

typedef struct {
    uint32_t frequency;                   // Frequency in Hz
    uint8_t  spreading_factor;            // 5-12
    uint16_t bandwidth;                   // 7, 10, 15, 20, 31, 41, 62, 125, 250, 500 kHz
    uint8_t  coding_rate;                 // 5-8 (4/5 to 4/8)
    uint8_t  sync_word;                   // Sync word
    uint16_t preamble_length;             // Preamble length in symbols
    uint8_t  power;                       // TX Power in dBm
    uint8_t  ramp_time;                   // Microseconds
    bool     crc_enabled;                 // CRC enabled/disabled
    bool     invert_iq;                   // Invert IQ enabled/disabled
    bool     low_data_rate_optimization;  // Low data rate optimization enabled/disabled
    bool     rx_boost;                    // Boosted RX gain (+3 dB sensitivity, +~2 mA)
    bool     use_dcdc;                    // Enable DC-DC converter
    bool     use_automatic_correction;    // Enable automatic frequency correction
} __attribute__((packed)) lora_protocol_config_params_t;

#define LORA_PROTOCOL_CONFIG_PARAMS_LEGACY_SIZE offsetof(lora_protocol_config_params_t, low_data_rate_optimization)

typedef struct {
    uint16_t             errors;
    lora_protocol_chip_t chip_type;
    char                 version_string[LORA_PROTOCOL_VERSION_STRING_LENGTH];
} __attribute__((packed)) lora_protocol_status_params_t;

// Per-packet RF stats from the SX126x.
// Conversion: rssi_dbm = -rssi_pkt_raw/2, snr_db = snr_pkt_raw/4, signal_rssi_dbm = -signal_rssi_pkt_raw/2.
typedef struct {
    uint8_t rssi_pkt_raw;
    int8_t  snr_pkt_raw;
    uint8_t signal_rssi_pkt_raw;
} __attribute__((packed)) lora_packet_stats_t;

// Response payload for GET_RSSI_INST (instant RSSI, e.g. for noise floor measurement).
typedef struct {
    uint8_t rssi_raw;  // dBm = -rssi_raw/2
} __attribute__((packed)) lora_protocol_rssi_inst_params_t;

typedef struct {
    lora_packet_stats_t stats;
    uint8_t             length;
    uint8_t             data[256];
} __attribute__((packed)) lora_protocol_lora_packet_t;

typedef struct {
    float last_frequency_error_hz;
    float local_oscillator_offset_hz;
    float applied_frequency_offset_hz;
} __attribute__((packed)) lora_protocol_get_frequency_offset_params_t;

typedef struct {
    float frequency_offset_hz;
} __attribute__((packed)) lora_protocol_set_frequency_offset_params_t;

typedef struct {
    uint32_t sequence_number;
    uint32_t type;  // lora_protocol_packet_type_t
} __attribute__((packed)) lora_protocol_header_t;

// LoRa radio handle

typedef struct {
    // Driver handle
    sx126x_handle_t driver_handle;

    // Output queue
    QueueHandle_t lora_packet_queue;

    // State
    SemaphoreHandle_t             lora_mutex;
    SemaphoreHandle_t             lora_transaction_semaphore;
    uint32_t                      lora_sequence_number;  // Transaction sequence number
    lora_protocol_config_params_t lora_config;           // Only used for local radios
    TaskHandle_t                  lora_task;

    // Single packet storage for transaction responses
    uint8_t lora_packet_buffer[sizeof(uint32_t) + 512];
    size_t  lora_packet_size;

    // Frequency offset
    float   last_frequency_error_hz;
    float   frequency_error_history[LORA_FREQUENCY_ERROR_HISTORY_LENGTH];  // Circular buffer for moving average
    uint8_t frequency_error_history_index;                                 // Next slot to write in the buffer
    uint8_t frequency_error_history_count;  // Number of valid samples (until buffer fills up)
    float   local_oscillator_offset_hz;     // Moving average of the frequency error over the last packets
    float   applied_frequency_offset_hz;    // Currently applied offset
} lora_handle_t;

// Functions

esp_err_t     lora_init_remote(lora_handle_t* handle, uint32_t packet_queue_size);
esp_err_t     lora_init_local(lora_handle_t* handle, uint32_t packet_queue_size, spi_host_device_t spi_host_id,
                              gpio_num_t nss, gpio_num_t reset, gpio_num_t dio1, gpio_num_t busy);
QueueHandle_t lora_get_packet_queue(lora_handle_t* handle);

esp_err_t lora_get_mode(lora_handle_t* handle, lora_protocol_mode_t* out_mode);
esp_err_t lora_set_mode(lora_handle_t* handle, const lora_protocol_mode_t mode);

esp_err_t lora_get_config(lora_handle_t* handle, lora_protocol_config_params_t* out_config);
esp_err_t lora_set_config(lora_handle_t* handle, const lora_protocol_config_params_t* config);

esp_err_t lora_get_status(lora_handle_t* handle, lora_protocol_status_params_t* out_status);

esp_err_t lora_send_packet(lora_handle_t* handle, const lora_protocol_lora_packet_t* packet);
esp_err_t lora_receive_packet(lora_handle_t* handle, lora_protocol_lora_packet_t* out_packet, TickType_t timeout);

esp_err_t lora_get_rssi_inst(lora_handle_t* handle, float* out_rssi);

esp_err_t lora_get_frequency_offset(lora_handle_t* handle, float* out_frequency_error,
                                    float* out_local_oscillator_offset, float* out_applied_offset);
esp_err_t lora_set_frequency_offset(lora_handle_t* handle, float offset);
