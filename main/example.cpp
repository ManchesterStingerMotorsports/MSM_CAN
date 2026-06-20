#include "MSM_CAN.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

static const char *TAG = "MSM_CAN_EXAMPLE";

static const gpio_num_t CAN_RX_GPIO = GPIO_NUM_5;
static const gpio_num_t CAN_TX_GPIO = GPIO_NUM_4;

static const uint16_t CALLBACK_RX_ID = 0x200;
static const uint16_t POLLING_RX_ID = 0x201;
static const uint16_t ONE_SHOT_TX_ID = 0x500;
static const uint16_t PERIODIC_TX_ID = 0x501;

// This function runs automatically when a frame arrives for CALLBACK_RX_ID.
// Keep callbacks short: copy or unpack the data, then return quickly.
static void handle_can_message(const MSM_CAN::RxFrame& frame)
{
    uint16_t value = MSM_CAN::unpack_u16(frame.data, 0);

    ESP_LOGI(TAG,
             "Callback received 0x%03X: value=%u timestamp=%lu",
             frame.id,
             static_cast<unsigned>(value),
             static_cast<unsigned long>(frame.timestamp_ms));
}

extern "C" void app_main(void)
{
    // Pick which CAN IDs this node should receive before starting the driver.
    MSM_CAN::set_hardware_filters(CALLBACK_RX_ID, POLLING_RX_ID);

    // Start the CAN driver on your chosen RX and TX pins.
    MSM_CAN::init(CAN_RX_GPIO, CAN_TX_GPIO);

    // Option 1: use a callback.
    // handle_can_message() will run whenever CALLBACK_RX_ID is received.
    MSM_CAN::subscribe(CALLBACK_RX_ID, handle_can_message);

    // Option 2: or instead, subscribe without a callback and just call get().
    // The library stores the latest frame for this ID until you read it.
    MSM_CAN::subscribe(POLLING_RX_ID);

    // Send one CAN frame now.
    MSM_CAN::TxFrame one_shot = {};
    one_shot.id = ONE_SHOT_TX_ID;
    MSM_CAN::pack_u16(one_shot.data, 0, 1234);
    MSM_CAN::pack_u16(one_shot.data, 2, 5678);
    MSM_CAN::send_msg(one_shot);

    // Send another CAN frame every 100 ms.
    MSM_CAN::TxFrame periodic = {};
    periodic.id = PERIODIC_TX_ID;
    MSM_CAN::pack_u16(periodic.data, 0, 0xAA55);
    MSM_CAN::schedule(periodic, 100);

    while (true)
    {
        // Polling example: get() returns the latest cached frame if one exists.
        // Calling get() again will return the same frame until a newer one arrives.
        MSM_CAN::RxFrame polled_frame = {};
        if (MSM_CAN::get(POLLING_RX_ID, polled_frame) == ESP_OK)
        {
            uint16_t value = MSM_CAN::unpack_u16(polled_frame.data, 0);

            ESP_LOGI(TAG,
                     "Polled 0x%03X: value=%u timestamp=%lu",
                     polled_frame.id,
                     static_cast<unsigned>(value),
                     static_cast<unsigned long>(polled_frame.timestamp_ms));
        }

        // If you only want to read each received frame once, use this instead:
        //
        // if (MSM_CAN::get_and_clear(POLLING_RX_ID, polled_frame) == ESP_OK)
        // {
        //     // This frame has now been consumed.
        // }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}
