#include "MSM_CAN.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "MSM_CAN_EXAMPLE";

static constexpr gpio_num_t CAN_RX_GPIO = GPIO_NUM_5;
static constexpr gpio_num_t CAN_TX_GPIO = GPIO_NUM_4;

static constexpr uint16_t CALLBACK_RX_ID = 0x200;
static constexpr uint16_t POLLING_RX_ID = 0x201;
static constexpr uint16_t ONE_SHOT_TX_ID = 0x500;
static constexpr uint16_t PERIODIC_TX_ID = 0x501;

static portMUX_TYPE s_callback_sample_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_callback_sample_pending = false;
static uint16_t s_callback_sample_value = 0;
static uint32_t s_callback_sample_timestamp_ms = 0;
static uint32_t s_last_polled_timestamp_ms = 0;

static MSM_CAN::TxFrame make_word_frame(uint16_t id,
                                        uint16_t first_word,
                                        uint16_t second_word)
{
    MSM_CAN::TxFrame frame = {};
    frame.id = id;
    MSM_CAN::pack_u16(frame.data, 0, first_word);
    MSM_CAN::pack_u16(frame.data, 2, second_word);
    return frame;
}

static void callback_rx_handler(const MSM_CAN::RxFrame& frame)
{
    if (frame.id != CALLBACK_RX_ID)
    {
        return;
    }

    const uint16_t value = MSM_CAN::unpack_u16(frame.data, 0);

    portENTER_CRITICAL(&s_callback_sample_mux);
    s_callback_sample_pending = true;
    s_callback_sample_value = value;
    s_callback_sample_timestamp_ms = frame.timestamp_ms;
    portEXIT_CRITICAL(&s_callback_sample_mux);
}

static void log_callback_sample_if_available()
{
    bool pending = false;
    uint16_t value = 0;
    uint32_t timestamp_ms = 0;

    portENTER_CRITICAL(&s_callback_sample_mux);
    pending = s_callback_sample_pending;
    if (pending)
    {
        value = s_callback_sample_value;
        timestamp_ms = s_callback_sample_timestamp_ms;
        s_callback_sample_pending = false;
    }
    portEXIT_CRITICAL(&s_callback_sample_mux);

    if (pending)
    {
        ESP_LOGI(TAG,
                 "Callback RX 0x%03X: value=%u timestamp_ms=%lu",
                 CALLBACK_RX_ID,
                 static_cast<unsigned>(value),
                 static_cast<unsigned long>(timestamp_ms));
    }
}

static void poll_latest_frame()
{
    MSM_CAN::RxFrame frame = {};
    // get() leaves the cached frame available; use get_and_clear() for
    // consume-on-read polling.
    const esp_err_t err = MSM_CAN::get(POLLING_RX_ID, frame);
    if (err == ESP_ERR_NOT_FOUND)
    {
        return;
    }

    ESP_ERROR_CHECK(err);

    if (frame.timestamp_ms == s_last_polled_timestamp_ms)
    {
        return;
    }
    s_last_polled_timestamp_ms = frame.timestamp_ms;

    ESP_LOGI(TAG,
             "Polled RX 0x%03X: value=%u timestamp_ms=%lu",
             POLLING_RX_ID,
             static_cast<unsigned>(MSM_CAN::unpack_u16(frame.data, 0)),
             static_cast<unsigned long>(frame.timestamp_ms));
}

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(MSM_CAN::set_hardware_filters(CALLBACK_RX_ID, POLLING_RX_ID));
    ESP_ERROR_CHECK(MSM_CAN::init(CAN_RX_GPIO, CAN_TX_GPIO));

    ESP_ERROR_CHECK(MSM_CAN::subscribe(CALLBACK_RX_ID, callback_rx_handler));
    ESP_ERROR_CHECK(MSM_CAN::subscribe(POLLING_RX_ID));

    MSM_CAN::TxFrame one_shot = make_word_frame(ONE_SHOT_TX_ID, 0x1234, 0x5678);
    ESP_ERROR_CHECK(MSM_CAN::send_msg(one_shot));

    MSM_CAN::TxFrame periodic = make_word_frame(PERIODIC_TX_ID, 0xAA55, 0x0102);
    ESP_ERROR_CHECK(MSM_CAN::schedule(periodic, 100));

    vTaskDelay(pdMS_TO_TICKS(500));

    periodic = make_word_frame(PERIODIC_TX_ID, 0xCC33, 0x0405);
    ESP_ERROR_CHECK(MSM_CAN::update_scheduled_payload(periodic));

    while (true)
    {
        log_callback_sample_if_available();
        poll_latest_frame();
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}
