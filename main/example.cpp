#include "MSM_CAN.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "MSM_CAN_EXAMPLE";

static volatile bool saw_msg_200 = false;
static uint8_t last_msg_200[8];
static volatile uint16_t latest_rx_value = 0;
static volatile uint32_t latest_rx_timestamp = 0;

static void can_callback(uint16_t id,
                         const uint8_t data[8],
                         uint32_t timestamp)
{
    if (id != 0x200)
    {
        return;
    }

    for (int i = 0; i < 8; i++)
    {
        last_msg_200[i] = data[i];
    }

    latest_rx_value = MSM_CAN::unpack_u16(data, 0);
    latest_rx_timestamp = timestamp;
    saw_msg_200 = true;
}

extern "C" void app_main(void)
{
    // Listen for one incoming ID and initialise the CAN driver.
    MSM_CAN::set_hardware_filters(0x200);
    MSM_CAN::init(GPIO_NUM_5, GPIO_NUM_4);
    MSM_CAN::subscribe(0x200, can_callback);

    // A subscription can also be callback-free. The latest packet is cached
    // internally and can be polled with MSM_CAN::get().
    MSM_CAN::subscribe(0x201);

    // Send a simple one-shot frame.
    uint8_t tx_data[8];
    MSM_CAN::clear_payload(tx_data);

    // Example of the helper pack functions for building a payload.
    MSM_CAN::pack_u16(tx_data, 0, 0x1234);
    MSM_CAN::pack_u16(tx_data, 2, 0x5678);
    MSM_CAN::pack_u8(tx_data, 4, 0x9A);

    MSM_CAN::send_msg(0x500, tx_data);

    // Example of the helper unpack functions for reading data back out.
    const uint16_t first_word = MSM_CAN::unpack_u16(tx_data, 0);
    const uint16_t second_word = MSM_CAN::unpack_u16(tx_data, 2);
    ESP_LOGI(TAG, "One-shot payload words: 0x%04X 0x%04X", first_word, second_word);

    // Schedule a second frame to go out every 100 ms.
    uint8_t periodic_data[8];
    MSM_CAN::clear_payload(periodic_data);
    MSM_CAN::pack_u16(periodic_data, 0, 0xAA55);
    MSM_CAN::pack_u16(periodic_data, 2, 0x0102);
    MSM_CAN::schedule(0x501, periodic_data, 100);

    vTaskDelay(pdMS_TO_TICKS(500));

    // Update the scheduled payload without changing its period.
    MSM_CAN::clear_payload(periodic_data);
    MSM_CAN::pack_u16(periodic_data, 0, 0xCC33);
    MSM_CAN::pack_u16(periodic_data, 2, 0x0405);
    MSM_CAN::update_scheduled_payload(0x501, periodic_data);

    vTaskDelay(pdMS_TO_TICKS(500));

    uint8_t cached_data[8];
    uint32_t cached_timestamp = 0;
    if (MSM_CAN::get(0x200, cached_data, &cached_timestamp) == ESP_OK)
    {
        ESP_LOGI(TAG,
                 "Cached RX for 0x200: value=%u timestamp=%lu",
                 static_cast<unsigned>(MSM_CAN::unpack_u16(cached_data, 0)),
                 static_cast<unsigned long>(cached_timestamp));
    }

    // Stop the periodic transmit.
    MSM_CAN::unschedule(0x501);

    // Nothing else is required here. The example has already shown one-shot TX,
    // scheduled TX, payload updates, un-scheduling, RX subscription, and the
    // pack/unpack helpers, so we just keep the task alive.
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
