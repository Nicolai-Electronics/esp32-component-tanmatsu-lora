#include "lora.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "include/lora.h"
#include "sx126x.h"

#if defined(CONFIG_IDF_TARGET_ESP32P4)
#include "esp_hosted.h"
#endif

static const char     TAG[]         = "lora";
static lora_handle_t* remote_handle = NULL;

static esp_err_t lora_transaction(lora_handle_t* handle, const uint8_t* request, size_t request_length,
                                  uint8_t* out_response, size_t* response_length, size_t max_response_length) {
    if (handle == NULL) {
        return ESP_FAIL;
    }

    if (!handle->lora_mutex || !handle->lora_transaction_semaphore || !handle->lora_packet_queue) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t result = ESP_FAIL;
    xSemaphoreTake(handle->lora_mutex, portMAX_DELAY);
    xSemaphoreTake(handle->lora_transaction_semaphore, 0);  // Clear semaphore
#if defined(CONFIG_IDF_TARGET_ESP32P4)
    result = esp_hosted_send_custom_data(1, (uint8_t*)request, request_length);
    if (result == ESP_OK) {
        if (xSemaphoreTake(handle->lora_transaction_semaphore, pdMS_TO_TICKS(2000)) == pdTRUE) {  // Wait for response
            if (handle->lora_packet_size <= max_response_length) {
                memcpy(out_response, handle->lora_packet_buffer, handle->lora_packet_size);
                *response_length = handle->lora_packet_size;
                result           = ESP_OK;
            } else {
                result = ESP_ERR_INVALID_SIZE;
            }
        } else {
            result = ESP_ERR_TIMEOUT;
        }
    }
#else
    result = ESP_OK;
#endif
    handle->lora_sequence_number++;
    xSemaphoreGive(handle->lora_mutex);
    return result;
}

#if defined(CONFIG_IDF_TARGET_ESP32P4)
static void lora_transaction_receive(uint32_t msg_id, const uint8_t* packet, size_t length) {
    lora_handle_t* handle = remote_handle;
    if (handle == NULL) {
        ESP_LOGW(TAG, "Received lora message ignored, no handle registered");
        return;
    }

    if (msg_id != 1) {
        ESP_LOGW(TAG, "Received lora message with unknown ID: %u", msg_id);
        return;
    }

    static lora_protocol_lora_packet_t lora_packet = {0};

    if (!handle->lora_mutex || !handle->lora_transaction_semaphore || !handle->lora_packet_queue) {
        ESP_LOGW(TAG, "Received lora message but lora not initialized");
        return;
    }
    if (length > sizeof(handle->lora_packet_buffer) || length < sizeof(lora_protocol_header_t)) {
        ESP_LOGW(TAG, "Received lora message but size incorrect");
        return;
    }
    lora_protocol_header_t* header = (lora_protocol_header_t*)packet;
    if (header->type == LORA_PROTOCOL_TYPE_PACKET_RX) {
        // Wire format: header || stats(3) || length(1) || data[length]
        size_t stats_size = sizeof(lora_packet_stats_t);
        if (length < sizeof(lora_protocol_header_t) + stats_size + sizeof(uint8_t)) {
            ESP_LOGW(TAG, "PACKET_RX too short: %u", length);
            return;
        }
        uint8_t const* stats_bytes = packet + sizeof(lora_protocol_header_t);
        uint8_t        payload_len = packet[sizeof(lora_protocol_header_t) + stats_size];
        size_t         data_offset = sizeof(lora_protocol_header_t) + stats_size + sizeof(uint8_t);
        if (length < data_offset + payload_len) {
            ESP_LOGW(TAG, "PACKET_RX truncated: %u < %u", length, (unsigned)(data_offset + payload_len));
            return;
        }
        lora_packet.stats.rssi_pkt_raw        = stats_bytes[0];
        lora_packet.stats.snr_pkt_raw         = (int8_t)stats_bytes[1];
        lora_packet.stats.signal_rssi_pkt_raw = stats_bytes[2];
        lora_packet.length                    = payload_len;
        memcpy(lora_packet.data, packet + data_offset, payload_len);
        xQueueSend(handle->lora_packet_queue, &lora_packet, 0);
    } else {
        // Response to a transaction
        memcpy(handle->lora_packet_buffer, packet, length);
        handle->lora_packet_size = length;
        xSemaphoreGive(handle->lora_transaction_semaphore);
    }
}
#endif

esp_err_t lora_init_remote(lora_handle_t* handle, uint32_t packet_queue_size) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (remote_handle != NULL) {
        return ESP_FAIL;
    }

#if !defined(CONFIG_IDF_TARGET_ESP32P4)
    return ESP_FAIL;
#endif

    handle->lora_mutex                 = xSemaphoreCreateMutex();
    handle->lora_transaction_semaphore = xSemaphoreCreateBinary();
    handle->lora_packet_queue          = xQueueCreate(packet_queue_size, sizeof(lora_protocol_lora_packet_t));
    if (handle->lora_mutex == NULL || handle->lora_transaction_semaphore == NULL || handle->lora_packet_queue == NULL) {
        if (handle->lora_mutex != NULL) {
            vSemaphoreDelete(handle->lora_mutex);
        }
        if (handle->lora_transaction_semaphore != NULL) {
            vSemaphoreDelete(handle->lora_transaction_semaphore);
        }
        if (handle->lora_packet_queue != NULL) {
            vQueueDelete(handle->lora_packet_queue);
        }
        return ESP_ERR_NO_MEM;
    }

#if defined(CONFIG_IDF_TARGET_ESP32P4)
    esp_hosted_register_custom_callback(1, lora_transaction_receive);
#endif

    remote_handle = handle;

    return ESP_OK;
}

static esp_err_t lora_initialize_radio(lora_handle_t* handle) {
    esp_err_t res = sx126x_set_op_mode_standby(&handle->driver_handle, false);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa radio to standby mode: %s", esp_err_to_name(res));
        return res;
    }

    char version_string[17] = {0};
    res = sx126x_read_version_string(&handle->driver_handle, version_string, sizeof(version_string));
    if (res != ESP_OK) {
        return res;
    }

    res = sx126x_clear_device_errors(&handle->driver_handle, NULL, NULL);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear LoRa radio device errors: %s", esp_err_to_name(res));
        return res;
    }

    const uint8_t testdata[] = {0x13, 0x37};
    res                      = sx126x_write_buffer(&handle->driver_handle, 0x00, testdata, sizeof(testdata));
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write data to LoRa radio: %s", esp_err_to_name(res));
        return res;
    }

    uint8_t read_data[sizeof(testdata)] = {0};
    res = sx126x_read_buffer(&handle->driver_handle, 0x00, read_data, sizeof(read_data));
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data from LoRa radio: %s", esp_err_to_name(res));
        return res;
    }

    if (memcmp(testdata, read_data, sizeof(testdata)) != 0) {
        ESP_LOGE(TAG, "LoRa radio initialization failed: Read data does not match written data %02x %02x", read_data[0],
                 read_data[1]);
        return ESP_FAIL;
    }

    res = sx126x_set_dio2_as_rf_switch_ctrl(&handle->driver_handle, true);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LoRa radio DIO2 as RF switch control: %s", esp_err_to_name(res));
        return res;
    }

    res = sx126x_set_packet_type(&handle->driver_handle, SX126X_PACKET_TYPE_LORA);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa radio packet type: %s", esp_err_to_name(res));
        return res;
    }

    res = sx126x_set_rx_tx_fallback_mode(&handle->driver_handle, SX126X_FALLBACK_MODE_STDBY_RC);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa radio RX/TX fallback mode: %s", esp_err_to_name(res));
        return res;
    }

    res = sx126x_set_cad_params(&handle->driver_handle, SX126X_CAD_ON_8_SYMB, 8 + 13, 10, false, 0xFFFFFF);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa radio CAD parameters: %s", esp_err_to_name(res));
        return res;
    }

    res = sx126x_clear_irq_status(&handle->driver_handle, 0);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear LoRa radio IRQ status: %s", esp_err_to_name(res));
        return res;
    }

    res = sx126x_calibrate(&handle->driver_handle, true, true, true, true, true, true, true);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calibrate LoRa radio: %s", esp_err_to_name(res));
        return res;
    }

    while (sx126x_is_busy(&handle->driver_handle)) {
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_LOGI(TAG, "Waiting for LoRa radio to be calibrated...");
    }

    uint16_t errors = 0;

    res = sx126x_get_device_errors(&handle->driver_handle, &errors);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get LoRa radio device errors: %s", esp_err_to_name(res));
        return res;
    }

    bool tcxo_detected = false;

    if (errors == SX126X_XOSC_START_ERR) {
        tcxo_detected = true;
        res           = sx126x_clear_device_errors(&handle->driver_handle, NULL, NULL);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to clear LoRa radio device errors: %s", esp_err_to_name(res));
            return res;
        }
        res = sx126x_set_dio3_as_txco_ctrl(&handle->driver_handle, 1.8, 5000);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure LoRa radio DIO3 as TCXO control: %s", esp_err_to_name(res));
            return res;
        }
        res = sx126x_calibrate(&handle->driver_handle, true, true, true, true, true, true, true);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to calibrate LoRa radio: %s", esp_err_to_name(res));
            return res;
        }
        while (sx126x_is_busy(&handle->driver_handle)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            ESP_LOGI(TAG, "Waiting for LoRa radio to be calibrated...");
        }
        res = sx126x_get_device_errors(&handle->driver_handle, &errors);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get LoRa radio device errors: %s", esp_err_to_name(res));
            return res;
        }
    }

    if (errors != 0) {
        ESP_LOGE(TAG, "LoRa radio device errors detected: 0x%04x", errors);

        if (errors & SX126X_ERROR_RC64K_CALIB_ERR) ESP_LOGE(TAG, "RC64K calibration error");
        if (errors & SX126X_ERROR_RC13M_CALIB_ERR) ESP_LOGE(TAG, "RC13M calibration error");
        if (errors & SX126X_ERROR_PLL_CALIB_ERR) ESP_LOGE(TAG, "PLL calibration error");
        if (errors & SX126X_ERROR_ADC_CALIB_ERR) ESP_LOGE(TAG, "ADC calibration error");
        if (errors & SX126X_ERROR_IMG_CALIB_ERR) ESP_LOGE(TAG, "Image calibration error");
        if (errors & SX126X_XOSC_START_ERR) ESP_LOGE(TAG, "XOSC start error");
        if (errors & SX126X_PLL_LOCK_ERR) ESP_LOGE(TAG, "PLL lock error");
        if (errors & SX126X_PA_RAMP_ERR) ESP_LOGE(TAG, "PA ramp error");
        return ESP_FAIL;
    }

    uint8_t chip_mode = 0;
    res               = sx126x_get_status(&handle->driver_handle, NULL, &chip_mode);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get LoRa radio status: %s", esp_err_to_name(res));
        return res;
    }

    if (chip_mode != SX126X_CHIP_MODE_STDBY_RC) {
        ESP_LOGE(TAG, "LoRa radio not in expected standby RC mode after initialization, mode: %d", chip_mode);
        return ESP_FAIL;
    }

    uint16_t irq_mask = SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_PREAMBLE_DETECTED |
                        SX126X_IRQ_HEADER_VALID | SX126X_IRQ_HEADER_ERROR | SX126X_IRQ_CRC_ERROR | SX126X_IRQ_CAD_DONE |
                        SX126X_IRQ_CAD_DETECTED | SX126X_IRQ_TIMEOUT;
    uint16_t dio1_mask = SX126X_IRQ_ALL;
    uint16_t dio2_mask = 0;  // DIO2 is used as RF switch control
    uint16_t dio3_mask = 0;  // DIO3 is used as TCXO control if applicable

    res = sx126x_set_dio_irq_params(&handle->driver_handle, irq_mask, dio1_mask, dio2_mask, dio3_mask);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa DIO IRQ parameters: %s", esp_err_to_name(res));
        return res;
    }

    // TxClampConfig workaround from datasheet
    uint8_t temp = 0;
    res          = sx126x_read_register(&handle->driver_handle, 0x08D8, &temp, 1);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read LoRa TxClampConfig register: %s", esp_err_to_name(res));
        return res;
    }
    temp |= 0x1E;
    res   = sx126x_write_register(&handle->driver_handle, 0x08D8, &temp, 1);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write LoRa TxClampConfig register: %s", esp_err_to_name(res));
        return res;
    }

    ESP_LOGI(TAG, "LoRa chip initialized (%s with %s)", version_string, tcxo_detected ? "TCXO" : "XOSC");

    return ESP_OK;
}

static void lora_radio_read_data(lora_handle_t* handle) {
    static lora_protocol_lora_packet_t lora_packet   = {0};
    uint8_t                            start_pointer = 0;
    esp_err_t res = sx126x_get_rx_buffer_status(&handle->driver_handle, &lora_packet.length, &start_pointer);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get LoRa RX buffer status: %s", esp_err_to_name(res));
        return;
    }
    if (lora_packet.length == 0) {
        ESP_LOGW(TAG, "Failed to read data from LoRa radio: available data length is zero");
        return;
    }

    res =
        sx126x_get_packet_status_lora(&handle->driver_handle, &lora_packet.stats.rssi_pkt_raw,
                                      (uint8_t*)&lora_packet.stats.snr_pkt_raw, &lora_packet.stats.signal_rssi_pkt_raw);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read packet status from LoRa radio: %s", esp_err_to_name(res));
        return;
    }

    res = sx126x_read_buffer(&handle->driver_handle, start_pointer, lora_packet.data, lora_packet.length);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read packet from LoRa radio: %s", esp_err_to_name(res));
        return;
    }

    if (xQueueSend(handle->lora_packet_queue, &lora_packet, 0) != pdPASS) {
        ESP_LOGW(TAG, "Incoming LoRa packet dropped, queue full");
    }
}

static void lora_radio_task(void* pvParameters) {
    lora_handle_t* handle = (lora_handle_t*)pvParameters;

    while (1) {
        esp_err_t res = sx126x_irq_wait(&handle->driver_handle, portMAX_DELAY);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Waiting for interrupt failed: %s", esp_err_to_name(res));
        }

        uint16_t interrupts     = 0;
        uint8_t  command_status = 0;
        uint8_t  chip_mode      = 0;
        res = sx126x_get_irq_status(&handle->driver_handle, &interrupts, &command_status, &chip_mode);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get LoRa IRQ status: %s", esp_err_to_name(res));
            continue;
        }

        // if (interrupts & SX126X_IRQ_TX_DONE) printf("Interrupt: TX done\r\n");
        // if (interrupts & SX126X_IRQ_RX_DONE) printf("Interrupt: RX done\r\n");
        if (interrupts & SX126X_IRQ_PREAMBLE_DETECTED) printf("Interrupt: preamble detected\r\n");
        if (interrupts & SX126X_IRQ_SYNC_WORD_VALID) printf("Interrupt: sync word valid\r\n");
        if (interrupts & SX126X_IRQ_HEADER_VALID) printf("Interrupt: header valid\r\n");
        if (interrupts & SX126X_IRQ_HEADER_ERROR) printf("Interrupt: header error\r\n");
        if (interrupts & SX126X_IRQ_CRC_ERROR) printf("Interrupt: crc error\r\n");
        if (interrupts & SX126X_IRQ_CAD_DONE) printf("Interrupt: cad done\r\n");
        if (interrupts & SX126X_IRQ_CAD_DETECTED) printf("Interrupt: cad detected\r\n");
        if (interrupts & SX126X_IRQ_TIMEOUT) printf("Interrupt: timeout\r\n");
        if (interrupts & SX126X_IRQ_LRFHSSHOP) printf("Interrupt: lrhsshop\r\n");

        res = sx126x_clear_irq_status(&handle->driver_handle, SX126X_IRQ_ALL);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to clear LoRa radio IRQ status: %s", esp_err_to_name(res));
            continue;
        }

        switch (command_status) {
            case SX126X_COMMAND_STATUS_DATA_AVAILABLE:
                printf("Data available!\r\n");
                lora_radio_read_data(handle);

                while (1) {
                    res = sx126x_set_op_mode_rx(&handle->driver_handle, 0);
                    if (res != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to set LoRa radio to RX mode: %s", esp_err_to_name(res));
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    } else {
                        break;
                    }
                }
                break;
            case SX126X_COMMAND_STATUS_TIMEOUT:
                printf("Operation timed out!\r\n");
                break;
            case SX126X_COMMAND_STATUS_INVALID:
                printf("Invalid operation!\r\n");
                break;
            case SX126X_COMMAND_STATUS_FAILED:
                printf("Operation failed!\r\n");
                break;
            case SX126X_COMMAND_STATUS_TX_DONE:
                xSemaphoreGive(handle->lora_transaction_semaphore);
                break;
            default:
                break;
        }

        /*switch (chip_mode) {
            case SX126X_CHIP_MODE_STDBY_RC:
                printf("Chip in STDBY_RC mode\r\n");
                break;
            case SX126X_CHIP_MODE_STDBY_XOSC:
                printf("Chip in STDBY_XOSC mode\r\n");
                break;
            case SX126X_CHIP_MODE_FS:
                printf("Chip in FS mode\r\n");
                break;
            case SX126X_CHIP_MODE_TX:
                printf("Chip in TX mode\r\n");
                break;
            case SX126X_CHIP_MODE_RX:
                printf("Chip in RX mode\r\n");
                break;
            default:
                break;
        }*/

        while (sx126x_is_busy(&handle->driver_handle)) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        uint16_t errors = 0;
        res             = sx126x_get_device_errors(&handle->driver_handle, &errors);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get LoRa radio device errors: %s", esp_err_to_name(res));
            continue;
        }

        if (errors != 0) {
            ESP_LOGE(TAG, "LoRa radio device errors detected: 0x%04x", errors);

            if (errors & SX126X_ERROR_RC64K_CALIB_ERR) ESP_LOGE(TAG, "RC64K calibration error");
            if (errors & SX126X_ERROR_RC13M_CALIB_ERR) ESP_LOGE(TAG, "RC13M calibration error");
            if (errors & SX126X_ERROR_PLL_CALIB_ERR) ESP_LOGE(TAG, "PLL calibration error");
            if (errors & SX126X_ERROR_ADC_CALIB_ERR) ESP_LOGE(TAG, "ADC calibration error");
            if (errors & SX126X_ERROR_IMG_CALIB_ERR) ESP_LOGE(TAG, "Image calibration error");
            if (errors & SX126X_XOSC_START_ERR) ESP_LOGE(TAG, "XOSC start error");
            if (errors & SX126X_PLL_LOCK_ERR) ESP_LOGE(TAG, "PLL lock error");
            if (errors & SX126X_PA_RAMP_ERR) ESP_LOGE(TAG, "PA ramp error");

            continue;
        }
    }

    vTaskDelete(NULL);
}

esp_err_t lora_init_local(lora_handle_t* handle, uint32_t packet_queue_size, spi_host_device_t spi_host_id,
                          gpio_num_t nss, gpio_num_t reset, gpio_num_t dio1, gpio_num_t busy) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Radio
    esp_err_t res = sx126x_init(&handle->driver_handle, spi_host_id, nss, reset, dio1, busy);
    if (res != ESP_OK) {
        return res;
    }

    res = lora_initialize_radio(handle);
    if (res != ESP_OK) {
        return res;
    }

    // Other
    handle->lora_mutex                 = xSemaphoreCreateMutex();
    handle->lora_transaction_semaphore = xSemaphoreCreateBinary();
    handle->lora_packet_queue          = xQueueCreate(packet_queue_size, sizeof(lora_protocol_lora_packet_t));
    if (handle->lora_mutex == NULL || handle->lora_transaction_semaphore == NULL || handle->lora_packet_queue == NULL) {
        if (handle->lora_mutex != NULL) {
            vSemaphoreDelete(handle->lora_mutex);
        }
        if (handle->lora_transaction_semaphore != NULL) {
            vSemaphoreDelete(handle->lora_transaction_semaphore);
        }
        if (handle->lora_packet_queue != NULL) {
            vQueueDelete(handle->lora_packet_queue);
        }
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(lora_radio_task, "lora", 4096, (void*)handle, 10, &handle->lora_task) != pdPASS) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

QueueHandle_t lora_get_packet_queue(lora_handle_t* handle) {
    if (handle == NULL) {
        return NULL;
    }
    return handle->lora_packet_queue;
}

esp_err_t lora_get_mode(lora_handle_t* handle, lora_protocol_mode_t* out_mode) {
    if (handle == NULL || out_mode == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle == remote_handle) {
        lora_protocol_header_t request = {
            .sequence_number = handle->lora_sequence_number,
            .type            = LORA_PROTOCOL_TYPE_GET_MODE,
        };
        uint8_t   response[sizeof(lora_protocol_header_t) + sizeof(lora_protocol_mode_params_t)] = {0};
        size_t    response_length                                                                = 0;
        esp_err_t result =
            lora_transaction(handle, (uint8_t*)&request, sizeof(request), response, &response_length, sizeof(response));
        if (result != ESP_OK) {
            return result;
        }
        if (response_length < sizeof(lora_protocol_header_t) + sizeof(lora_protocol_mode_params_t)) {
            ESP_LOGE(TAG, "Invalid response length: %u\r\n", response_length);
            return ESP_FAIL;
        }
        lora_protocol_header_t* header = (lora_protocol_header_t*)response;
        if (header->sequence_number != request.sequence_number) {
            ESP_LOGE(TAG, "Invalid response sequence number: %u\r\n", header->sequence_number);
            return ESP_FAIL;
        }
        if (header->type != LORA_PROTOCOL_TYPE_GET_MODE) {
            ESP_LOGE(TAG, "Invalid response type: %u\r\n", header->type);
            return ESP_FAIL;
        }
        lora_protocol_mode_params_t* params = (lora_protocol_mode_params_t*)(response + sizeof(lora_protocol_header_t));
        *out_mode                           = params->mode;
    } else {
        uint8_t   chip_mode = 0;
        esp_err_t res       = sx126x_get_status(&handle->driver_handle, NULL, &chip_mode);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get LoRa chip mode: %s", esp_err_to_name(res));
            return ESP_FAIL;
        }

        switch (chip_mode) {
            case SX126X_CHIP_MODE_STDBY_RC:
                *out_mode = LORA_PROTOCOL_MODE_STANDBY_RC;
                break;
            case SX126X_CHIP_MODE_STDBY_XOSC:
                *out_mode = LORA_PROTOCOL_MODE_STANDBY_XOSC;
                break;
            case SX126X_CHIP_MODE_FS:
                *out_mode = LORA_PROTOCOL_MODE_FS;
                break;
            case SX126X_CHIP_MODE_TX:
                *out_mode = LORA_PROTOCOL_MODE_TX;
                break;
            case SX126X_CHIP_MODE_RX:
                *out_mode = LORA_PROTOCOL_MODE_RX;
                break;
            default:
                ESP_LOGW(TAG, "Unknown LoRa chip mode: %d", chip_mode);
                *out_mode = LORA_PROTOCOL_MODE_UNKNOWN;
                break;
        }
    }
    return ESP_OK;
}

esp_err_t lora_set_mode(lora_handle_t* handle, const lora_protocol_mode_t mode) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle == remote_handle) {
        size_t                  request_length = sizeof(lora_protocol_header_t) + sizeof(lora_protocol_mode_params_t);
        uint8_t                 request[request_length];
        lora_protocol_header_t* header      = (lora_protocol_header_t*)request;
        header->sequence_number             = handle->lora_sequence_number;
        header->type                        = LORA_PROTOCOL_TYPE_SET_MODE;
        lora_protocol_mode_params_t* params = (lora_protocol_mode_params_t*)(request + sizeof(lora_protocol_header_t));
        params->mode                        = mode;
        uint8_t   response[sizeof(lora_protocol_header_t)] = {0};
        size_t    response_length                          = 0;
        esp_err_t result =
            lora_transaction(handle, request, request_length, response, &response_length, sizeof(response));
        if (result != ESP_OK) {
            return result;
        }
        if (response_length < sizeof(lora_protocol_header_t)) {
            ESP_LOGE(TAG, "Invalid response length: %u\r\n", response_length);
            return ESP_FAIL;
        }
        lora_protocol_header_t* response_header = (lora_protocol_header_t*)response;
        if (response_header->sequence_number != header->sequence_number) {
            ESP_LOGE(TAG, "Invalid response sequence number: %u\r\n", response_header->sequence_number);
            return ESP_FAIL;
        }
        if (response_header->type != LORA_PROTOCOL_TYPE_ACK) {
            ESP_LOGE(TAG, "Invalid response type: %u\r\n", response_header->type);
            return ESP_FAIL;
        }
    } else {
        switch (mode) {
            case LORA_PROTOCOL_MODE_STANDBY_RC:
                return sx126x_set_op_mode_standby(&handle->driver_handle, false);
            case LORA_PROTOCOL_MODE_STANDBY_XOSC:
                return sx126x_set_op_mode_standby(&handle->driver_handle, true);
            case LORA_PROTOCOL_MODE_FS:
                return sx126x_set_op_mode_fs(&handle->driver_handle);
            case LORA_PROTOCOL_MODE_TX:
                ESP_LOGE(TAG, "Can not set TX mode directly, host must send data to transmit");
                return ESP_ERR_INVALID_ARG;
            case LORA_PROTOCOL_MODE_RX:
                return sx126x_set_op_mode_rx(&handle->driver_handle, 0);
            default:
                ESP_LOGW(TAG, "Unknown LoRa mode requested: %d", mode);
                return ESP_ERR_INVALID_ARG;
        }
    }
    return ESP_OK;
}

esp_err_t lora_get_config(lora_handle_t* handle, lora_protocol_config_params_t* out_config) {
    if (handle == NULL || out_config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle == remote_handle) {
        lora_protocol_header_t request = {
            .sequence_number = handle->lora_sequence_number,
            .type            = LORA_PROTOCOL_TYPE_GET_CONFIG,
        };
        uint8_t   response[sizeof(lora_protocol_header_t) + sizeof(lora_protocol_config_params_t)] = {0};
        size_t    response_length                                                                  = 0;
        esp_err_t result =
            lora_transaction(handle, (uint8_t*)&request, sizeof(request), response, &response_length, sizeof(response));
        if (result != ESP_OK) {
            return result;
        }
        if (response_length < sizeof(lora_protocol_header_t) + sizeof(lora_protocol_config_params_t)) {
            ESP_LOGE(TAG, "Invalid response length: %u\r\n", response_length);
            return ESP_FAIL;
        }
        lora_protocol_header_t* header = (lora_protocol_header_t*)response;
        if (header->sequence_number != request.sequence_number) {
            ESP_LOGE(TAG, "Invalid response sequence number: %u\r\n", header->sequence_number);
            return ESP_FAIL;
        }
        if (header->type != LORA_PROTOCOL_TYPE_GET_CONFIG) {
            ESP_LOGE(TAG, "Invalid response type: %u\r\n", header->type);
            return ESP_FAIL;
        }
        lora_protocol_config_params_t* params =
            (lora_protocol_config_params_t*)(response + sizeof(lora_protocol_header_t));
        memcpy(out_config, params, sizeof(lora_protocol_config_params_t));
    } else {
        memcpy(out_config, &handle->lora_config, sizeof(lora_protocol_config_params_t));
    }
    return ESP_OK;
}

static esp_err_t lora_radio_apply_config(lora_handle_t* handle, const lora_protocol_config_params_t* config_params) {
    esp_err_t res = sx126x_set_regulator_mode(&handle->driver_handle, true);  // Use DC-DC
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa regulator mode to DC-DC: %s", esp_err_to_name(res));
        return res;
    }

    res = sx126x_set_rf_frequency(&handle->driver_handle, config_params->frequency);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa RF frequency: %s", esp_err_to_name(res));
        return res;
    }

    sx126x_lora_spreading_factor_t spreading_factor;
    switch (config_params->spreading_factor) {
        case 5:
            spreading_factor = SX126X_LORA_SPREADING_FACTOR_5;
            break;
        case 6:
            spreading_factor = SX126X_LORA_SPREADING_FACTOR_6;
            break;
        case 7:
            spreading_factor = SX126X_LORA_SPREADING_FACTOR_7;
            break;
        case 8:
            spreading_factor = SX126X_LORA_SPREADING_FACTOR_8;
            break;
        case 9:
            spreading_factor = SX126X_LORA_SPREADING_FACTOR_9;
            break;
        case 10:
            spreading_factor = SX126X_LORA_SPREADING_FACTOR_10;
            break;
        case 11:
            spreading_factor = SX126X_LORA_SPREADING_FACTOR_11;
            break;
        case 12:
            spreading_factor = SX126X_LORA_SPREADING_FACTOR_12;
            break;
        default:
            ESP_LOGW(TAG, "Invalid spreading factor: %d", config_params->spreading_factor);
            return ESP_ERR_INVALID_ARG;
    }

    sx126x_lora_bandwidth_t bandwidth;
    switch (config_params->bandwidth) {
        case 7:
            bandwidth = SX126X_LORA_BANDWIDTH_7;
            break;
        case 10:
            bandwidth = SX126X_LORA_BANDWIDTH_10;
            break;
        case 15:
            bandwidth = SX126X_LORA_BANDWIDTH_15;
            break;
        case 20:
            bandwidth = SX126X_LORA_BANDWIDTH_20;
            break;
        case 31:
            bandwidth = SX126X_LORA_BANDWIDTH_31;
            break;
        case 41:
            bandwidth = SX126X_LORA_BANDWIDTH_41;
            break;
        case 62:
            bandwidth = SX126X_LORA_BANDWIDTH_62;
            break;
        case 125:
            bandwidth = SX126X_LORA_BANDWIDTH_125;
            break;
        case 250:
            bandwidth = SX126X_LORA_BANDWIDTH_250;
            break;
        case 500:
            bandwidth = SX126X_LORA_BANDWIDTH_500;
            break;
        default:
            ESP_LOGW(TAG, "Invalid bandwidth: %d kHz", config_params->bandwidth);
            return ESP_ERR_INVALID_ARG;
    }

    sx126x_lora_coding_rate_t coding_rate;
    switch (config_params->coding_rate) {
        case 5:
            coding_rate = SX126X_LORA_CODING_RATE_4_5;
            break;
        case 6:
            coding_rate = SX126X_LORA_CODING_RATE_4_6;
            break;
        case 7:
            coding_rate = SX126X_LORA_CODING_RATE_4_7;
            break;
        case 8:
            coding_rate = SX126X_LORA_CODING_RATE_4_8;
            break;
        default:
            ESP_LOGW(TAG, "Invalid coding rate: 4/%d", config_params->coding_rate);
            return ESP_ERR_INVALID_ARG;
    }

    res = sx126x_set_modulation_params_lora(&handle->driver_handle, spreading_factor, bandwidth, coding_rate, true,
                                            config_params->low_data_rate_optimization);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa modulation parameters: %s", esp_err_to_name(res));
        return res;
    }

    res = sx126x_set_packet_params_lora_variable_length(&handle->driver_handle, config_params->preamble_length,
                                                        config_params->crc_enabled, config_params->invert_iq);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa packet parameters: %s", esp_err_to_name(res));
        return res;
    }

    res = sx126x_set_sync_word(&handle->driver_handle, config_params->sync_word);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa sync word and control bits: %s", esp_err_to_name(res));
        return res;
    }

    uint8_t pa_duty_cycle = 0x04;
    uint8_t hp_max        = 0x07;   // +22 dBm
    bool    is_sx1261     = false;  // True for SX1261, false for SX1262 and SX1268

    res = sx126x_set_pa_config(&handle->driver_handle, pa_duty_cycle, hp_max, is_sx1261);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa PA configuration: %s", esp_err_to_name(res));
        return res;
    }

    bool pa_is_high_power = true;

    res =
        sx126x_set_tx_params(&handle->driver_handle, config_params->power, pa_is_high_power, config_params->ramp_time);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa TX parameters: %s", esp_err_to_name(res));
        return res;
    }

    res = sx126x_set_buffer_base_address(&handle->driver_handle, 0x00, 0x00);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa buffer base address: %s", esp_err_to_name(res));
        return res;
    }

    res = sx126x_stop_timer_on_preamble(&handle->driver_handle, false);  // Stop timer on syncword
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LoRa stop timer on preamble: %s", esp_err_to_name(res));
        return res;
    }

    res = sx126x_set_lora_symb_num_timeout(&handle->driver_handle, 0);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa symbol number timeout: %s", esp_err_to_name(res));
        return res;
    }

    // RX gain mode (SX1262 datasheet section 9.6, RxGain register 0x08AC):
    // 0x96 = boosted (+3 dB sensitivity, +~2 mA), 0x94 = power-saving default.
    uint8_t rx_gain = config_params->rx_boost ? 0x96 : 0x94;
    res             = sx126x_write_register(&handle->driver_handle, 0x08AC, &rx_gain, 1);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa RX gain: %s", esp_err_to_name(res));
        return res;
    }

    return ESP_OK;
}

static uint32_t lora_calculate_tx_time_ms(const lora_protocol_config_params_t* config, uint8_t payload_length) {
    if (config == NULL || config->bandwidth == 0 || config->spreading_factor < 5 || config->spreading_factor > 12) {
        return 10000;  // Invalid configuration will yield a 10 second timeout
    }

    uint8_t  sf   = config->spreading_factor;
    uint16_t bw   = config->bandwidth;        // kHz (7, 10, 15, 20, 31, 41, 62, 125, 250, 500)
    uint8_t  cr   = config->coding_rate - 4;  // 1-4 representing 4/5 to 4/8
    uint8_t  ldro = config->low_data_rate_optimization ? 1 : 0;
    uint8_t  crc  = config->crc_enabled ? 1 : 0;

    // LoRa payload symbol count (SX126x datasheet, section 6.1.4):
    // n_payload = 8 + max(0, ceil((8*PL - 4*SF + 28 + 16*CRC) / (4*(SF - 2*LDRO))) * (CR + 4))
    int32_t num = (int32_t)(8 * payload_length) - (int32_t)(4 * sf) + 28 + (int32_t)(16 * crc);
    int32_t den = 4 * ((int32_t)sf - 2 * (int32_t)ldro);

    uint32_t payload_symbols = 8;
    if (num > 0 && den > 0) {
        payload_symbols += (uint32_t)((num + den - 1) / den) * (uint32_t)(cr + 4);
    }

    // Preamble is (preamble_length + 4.25) symbols; round up to preamble_length + 5.
    uint32_t total_symbols = (uint32_t)config->preamble_length + 5 + payload_symbols;

    // Symbol duration: T_s = 2^SF / BW_kHz  (result in ms)
    // ToA (ms) = total_symbols * 2^SF * 1000 / BW_kHz
    // Use uint64_t to avoid overflow with large preambles or high SF.
    uint64_t t_ms = ((uint64_t)total_symbols * ((uint64_t)1 << sf) * 1000ULL) / (uint64_t)bw;

    return (t_ms > UINT32_MAX) ? UINT32_MAX : (uint32_t)t_ms;
}

esp_err_t lora_set_config(lora_handle_t* handle, const lora_protocol_config_params_t* config) {
    if (handle == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle == remote_handle) {
        size_t                  request_length = sizeof(lora_protocol_header_t) + sizeof(lora_protocol_config_params_t);
        uint8_t                 request[request_length];
        lora_protocol_header_t* header = (lora_protocol_header_t*)request;
        header->sequence_number        = handle->lora_sequence_number;
        header->type                   = LORA_PROTOCOL_TYPE_SET_CONFIG;
        lora_protocol_config_params_t* params =
            (lora_protocol_config_params_t*)(request + sizeof(lora_protocol_header_t));
        memcpy(params, config, sizeof(lora_protocol_config_params_t));
        uint8_t   response[sizeof(lora_protocol_header_t)] = {0};
        size_t    response_length                          = 0;
        esp_err_t result =
            lora_transaction(handle, request, request_length, response, &response_length, sizeof(response));
        if (result != ESP_OK) {
            return result;
        }
        if (response_length < sizeof(lora_protocol_header_t)) {
            ESP_LOGE(TAG, "Invalid response length: %u\r\n", response_length);
            return ESP_FAIL;
        }
        lora_protocol_header_t* response_header = (lora_protocol_header_t*)response;
        if (response_header->sequence_number != header->sequence_number) {
            ESP_LOGE(TAG, "Invalid response sequence number: %u\r\n", response_header->sequence_number);
            return ESP_FAIL;
        }
        if (response_header->type != LORA_PROTOCOL_TYPE_ACK) {
            ESP_LOGE(TAG, "Invalid response type: %u\r\n", response_header->type);
            return ESP_FAIL;
        }
    } else {
        memcpy(&handle->lora_config, config, sizeof(lora_protocol_config_params_t));
        return lora_radio_apply_config(handle, config);
    }
    return ESP_OK;
}

esp_err_t lora_get_status(lora_handle_t* handle, lora_protocol_status_params_t* out_status) {
    if (handle == NULL || out_status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle == remote_handle) {
        lora_protocol_header_t request = {
            .sequence_number = handle->lora_sequence_number,
            .type            = LORA_PROTOCOL_TYPE_GET_STATUS,
        };
        uint8_t   response[sizeof(lora_protocol_header_t) + sizeof(lora_protocol_status_params_t)] = {0};
        size_t    response_length                                                                  = 0;
        esp_err_t result =
            lora_transaction(handle, (uint8_t*)&request, sizeof(request), response, &response_length, sizeof(response));
        if (result != ESP_OK) {
            return result;
        }
        if (response_length < sizeof(lora_protocol_header_t) + sizeof(lora_protocol_status_params_t)) {
            ESP_LOGE(TAG, "Invalid response length: %u\r\n", response_length);
            return ESP_FAIL;
        }
        lora_protocol_header_t* header = (lora_protocol_header_t*)response;
        if (header->sequence_number != request.sequence_number) {
            ESP_LOGE(TAG, "Invalid response sequence number: %u\r\n", header->sequence_number);
            return ESP_FAIL;
        }
        if (header->type != LORA_PROTOCOL_TYPE_GET_STATUS) {
            ESP_LOGE(TAG, "Invalid response type: %u\r\n", header->type);
            return ESP_FAIL;
        }
        lora_protocol_status_params_t* params =
            (lora_protocol_status_params_t*)(response + sizeof(lora_protocol_header_t));
        memcpy(out_status, params, sizeof(lora_protocol_status_params_t));
    } else {
        esp_err_t res = sx126x_read_version_string(&handle->driver_handle, out_status->version_string,
                                                   LORA_PROTOCOL_VERSION_STRING_LENGTH);

        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read LoRa version string: %s", esp_err_to_name(res));
            snprintf(out_status->version_string, LORA_PROTOCOL_VERSION_STRING_LENGTH, "UNKNOWN");
            return ESP_FAIL;
        }

        if (memcmp(out_status->version_string, "SX1268", strlen("SX1268")) == 0) {
            out_status->chip_type = LORA_PROTOCOL_CHIP_SX1268;
        } else {
            out_status->chip_type = LORA_PROTOCOL_CHIP_SX1262;
        }

        uint16_t errors = 0;
        res             = sx126x_get_device_errors(&handle->driver_handle, &errors);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get LoRa radio device errors: %s", esp_err_to_name(res));
            return ESP_FAIL;
        }
        out_status->errors = errors;
    }
    return ESP_OK;
}

esp_err_t lora_send_packet(lora_handle_t* handle, const lora_protocol_lora_packet_t* packet) {
    if (handle == NULL || packet == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle == remote_handle) {
        size_t                  request_length = sizeof(lora_protocol_header_t) + packet->length;
        uint8_t                 request[request_length];
        lora_protocol_header_t* header = (lora_protocol_header_t*)request;
        header->sequence_number        = handle->lora_sequence_number;
        header->type                   = LORA_PROTOCOL_TYPE_PACKET_TX;
        uint8_t* data_ptr              = (uint8_t*)(request + sizeof(lora_protocol_header_t));
        memcpy(data_ptr, packet->data, packet->length);
        uint8_t   response[sizeof(lora_protocol_header_t)] = {0};
        size_t    response_length                          = 0;
        esp_err_t result =
            lora_transaction(handle, request, request_length, response, &response_length, sizeof(response));
        if (result != ESP_OK) {
            return result;
        }
        if (response_length < sizeof(lora_protocol_header_t)) {
            ESP_LOGE(TAG, "Invalid response length: %u\r\n", response_length);
            return ESP_FAIL;
        }
        lora_protocol_header_t* response_header = (lora_protocol_header_t*)response;
        if (response_header->sequence_number != header->sequence_number) {
            ESP_LOGE(TAG, "Invalid response sequence number: %u\r\n", response_header->sequence_number);
            return ESP_FAIL;
        }
        if (response_header->type != LORA_PROTOCOL_TYPE_ACK) {
            ESP_LOGE(TAG, "Invalid response type: %u\r\n", response_header->type);
            return ESP_FAIL;
        }
    } else {
        // Check packet size limits
        if (handle->lora_config.coding_rate > 4 && packet->length < 8) {
            ESP_LOGW(TAG, "Packet length %d is too short for coding rate 8/%d, minimum is 8 bytes", packet->length,
                     handle->lora_config.coding_rate);
            return ESP_ERR_INVALID_SIZE;
        }

        if (handle->lora_config.crc_enabled && packet->length > 253) {
            ESP_LOGW(TAG, "Packet length %d is too long to include CRC when CRC is enabled, maximum is 253 bytes",
                     packet->length);
            return ESP_ERR_INVALID_SIZE;
        }

        // Read current mode to restore RX mode after transmission if needed
        uint8_t   chip_mode = 0;
        esp_err_t res       = sx126x_get_status(&handle->driver_handle, NULL, &chip_mode);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get LoRa chip mode: %s", esp_err_to_name(res));
            return res;
        }

        // Stop receiving, leave oscillator on
        res = sx126x_set_op_mode_standby(&handle->driver_handle, false);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set LoRa to standby mode before transmission: %s", esp_err_to_name(res));
            return res;
        }

        // Set packet type to LoRa
        res = sx126x_set_packet_type(&handle->driver_handle, SX126X_PACKET_TYPE_LORA);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set packet type to LoRa: %s", esp_err_to_name(res));
            return res;
        }

        // Set buffer base address
        res = sx126x_set_buffer_base_address(&handle->driver_handle, 0x00, 0x00);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set LoRa buffer base address: %s", esp_err_to_name(res));
            return res;
        }

        // Write packet data to buffer
        res = sx126x_write_buffer(&handle->driver_handle, 0x00, packet->data, packet->length);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write packet data to LoRa buffer: %s", esp_err_to_name(res));
            return res;
        }

        // Set packet parameters
        res = sx126x_set_packet_params_lora(&handle->driver_handle, handle->lora_config.preamble_length, false,
                                            packet->length, handle->lora_config.crc_enabled,
                                            handle->lora_config.invert_iq);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set LoRa packet parameters for transmission: %s", esp_err_to_name(res));
            return res;
        }

        // Clear TX done semaphore before starting transmission
        xSemaphoreTake(handle->lora_transaction_semaphore, 0);

        // Start transmit
        res = sx126x_set_op_mode_tx(&handle->driver_handle, 0);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set LoRa to TX mode: %s", esp_err_to_name(res));
            return res;
        }

        // Wait for transmission to complete (wait for TX_DONE event)
        uint32_t tx_time_ms = lora_calculate_tx_time_ms(&handle->lora_config, packet->length);
        uint32_t timeout_ms = (uint32_t)((uint64_t)tx_time_ms * 5 / 4 + 500);  // 25% margin + 500 ms overhead
        if (xSemaphoreTake(handle->lora_transaction_semaphore, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
            ESP_LOGE(TAG, "Timeout waiting for LoRa transmission to complete");
            return ESP_ERR_TIMEOUT;
        }

        // Restore packet parameters for receiving
        res = sx126x_set_packet_params_lora_variable_length(&handle->driver_handle, handle->lora_config.preamble_length,
                                                            handle->lora_config.crc_enabled,
                                                            handle->lora_config.invert_iq);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to restore LoRa packet parameters after transmission: %s", esp_err_to_name(res));
            return res;
        }

        // Restore chip mode (RX or standby)
        if (chip_mode == SX126X_CHIP_MODE_RX) {
            res = sx126x_set_op_mode_rx(&handle->driver_handle, 0);
            if (res != ESP_OK) {
                ESP_LOGE(TAG, "Failed to restore LoRa RX mode after transmission: %s", esp_err_to_name(res));
                return res;
            }
        } else {
            res = sx126x_set_op_mode_standby(&handle->driver_handle, false);
            if (res != ESP_OK) {
                ESP_LOGE(TAG, "Failed to restore LoRa standby mode after transmission: %s", esp_err_to_name(res));
                return res;
            }
        }
    }
    return ESP_OK;
}

esp_err_t lora_receive_packet(lora_handle_t* handle, lora_protocol_lora_packet_t* out_packet, TickType_t timeout) {
    if (handle == NULL || out_packet == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return xQueueReceive(handle->lora_packet_queue, out_packet, timeout) == pdTRUE ? ESP_OK : ESP_FAIL;
}

esp_err_t lora_get_rssi_inst(lora_handle_t* handle, float* out_rssi) {
    if (handle == NULL || out_rssi == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (handle == remote_handle) {
        lora_protocol_header_t request = {
            .sequence_number = handle->lora_sequence_number,
            .type            = LORA_PROTOCOL_TYPE_GET_RSSI_INST,
        };
        uint8_t   response[sizeof(lora_protocol_header_t) + sizeof(lora_protocol_rssi_inst_params_t)] = {0};
        size_t    response_length                                                                     = 0;
        esp_err_t result =
            lora_transaction(handle, (uint8_t*)&request, sizeof(request), response, &response_length, sizeof(response));
        if (result != ESP_OK) {
            return result;
        }
        lora_protocol_header_t* header = (lora_protocol_header_t*)response;
        if (header->sequence_number != request.sequence_number) {
            ESP_LOGE(TAG, "RSSI inst: response with unexpected sequence number %u", header->sequence_number);
            return ESP_FAIL;
        }
        if (header->type == LORA_PROTOCOL_TYPE_NACK) {
            ESP_LOGE(TAG, "RSSI inst: received error response");
            return ESP_FAIL;
        }
        if (header->type != LORA_PROTOCOL_TYPE_GET_RSSI_INST) {
            ESP_LOGE(TAG, "RSSI inst: response with unexpected type %u", header->type);
            return ESP_FAIL;
        }
        if (response_length < sizeof(lora_protocol_header_t) + sizeof(lora_protocol_rssi_inst_params_t)) {
            ESP_LOGE(TAG, "RSSI inst: response with unexpected length");
            return ESP_FAIL;
        }
        lora_protocol_rssi_inst_params_t* params =
            (lora_protocol_rssi_inst_params_t*)(response + sizeof(lora_protocol_header_t));
        *out_rssi = (params->rssi_raw - 0.5f) / -2.0f;
        return ESP_OK;
    } else {
        return sx126x_get_rssi_inst(&handle->driver_handle, out_rssi);
    }
}
