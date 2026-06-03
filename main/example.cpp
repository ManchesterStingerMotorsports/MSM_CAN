#include "MSM_CAN.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "MSM_CAN_EXAMPLE";

// 1. SIMPLE CALLBACK: This runs automatically when message 0x200 arrives
static void my_can_callback(uint16_t id, const uint8_t data[8], uint32_t timestamp)
{
    ESP_LOGI(TAG, "Callback triggered! Received message ID: 0x%03X", id);
}

extern "C" void app_main(void)
{
    // 2. INITIALIZATION
    MSM_CAN::set_hardware_filters(0x200);
    MSM_CAN::init(GPIO_NUM_5, GPIO_NUM_4);

    // Demonstrate Callback-based receiving (RX)
    MSM_CAN::subscribe(0x200, my_can_callback);

    // Demonstrate Polling-based receiving (RX) - Subscribe without a callback
    MSM_CAN::subscribe(0x201);

    // 3. THE MAIN LOOP
    while (true)
    {
        // A. DEMONSTRATE SENDING (TX)
        uint8_t tx_data[8];
        MSM_CAN::clear_payload(tx_data);
        MSM_CAN::pack_u16(tx_data, 0, 0x1234); // Pack some basic sample data
        
        ESP_LOGI(TAG, "Sending data frame to ID 0x500...");
        MSM_CAN::send_msg(0x500, tx_data);


        // B. DEMONSTRATE POLLING (RX via get)
        uint8_t rx_data[8];
        uint32_t rx_timestamp = 0;
        
        // If a message with ID 0x201 is waiting for us, grab it!
        if (MSM_CAN::get(0x201, rx_data, &rx_timestamp) == ESP_OK)
        {
            uint16_t value = MSM_CAN::unpack_u16(rx_data, 0);
            ESP_LOGI(TAG, "Polling Success! ID 0x201 data value: 0x%04X", value);
        }


        // Wait 1 second before repeating the loop
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}