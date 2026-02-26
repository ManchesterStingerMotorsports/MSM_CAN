#include "MSM_CAN.hpp"
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_err.h"
#include <algorithm>

namespace MSM_CAN
{

    struct RxPkt
    {
        uint16_t id;
        uint8_t data[8];
    };

    static QueueHandle_t g_rx_queue = nullptr;
    static constexpr size_t RX_QUEUE_DEPTH = 32;

    struct SubEntry
    {
        bool in_use; 
        uint16_t id;
        void (*callback)(uint16_t,
                         const uint8_t[8],
                         uint32_t);
      
    };

    static constexpr int MAX_SUBS = 64;           // subscription lookup table doesnt use dynamic memory
    static constexpr uint32_t TX_TIMEOUT_MS = 10; // transmit timeout
    static constexpr uint32_t BITRATE = 1000000;  // 1Mbit bus
    static constexpr uint8_t TX_QUEUE_DEPTH = 8;
    static constexpr uint8_t FAIL_RETRY_CNT = 3;

    static constexpr uint16_t TX_RANGE1_START = 0x100;
    static constexpr uint16_t TX_RANGE1_END = 0x1FF; // allowed high priority and low priority ID ranges for transmitting
    static constexpr uint16_t TX_RANGE2_START = 0x500;
    static constexpr uint16_t TX_RANGE2_END = 0x5FF;

    static constexpr uint16_t RX_TASK_STACK = 4096;

    static bool g_initialised = false;               // blocks especially silly minions from calling send_msg() or subscribe() before init().
    static SubEntry g_subs[MAX_SUBS];                // internal subscription table
    static SemaphoreHandle_t g_subs_mutex = nullptr; // prevents race conditions on g_subs[]

    static twai_node_handle_t g_node = nullptr;

    static twai_mask_filter_config_t s_filter_cfg =
        {
            .id = 0xFFFFFFFFu,
            .mask = 0xFFFFFFFFu,
            .is_ext = false,
    };

    static uint32_t highest_set_bit_index_u32(uint32_t x) // helper for filter range
    {
        uint32_t idx = 0;
        while (x >>= 1)
        {
            idx++;
        }
        return idx;
    }

    static inline bool is_allowed_tx_id(uint16_t id)
    {
        return ((id >= TX_RANGE1_START && id <= TX_RANGE1_END) ||
                (id >= TX_RANGE2_START && id <= TX_RANGE2_END));
    }

    static bool is_allowed_rx_id(uint16_t id)
    {
        const uint32_t masked_id = (id & s_filter_cfg.mask);
        const uint32_t masked_filter = (s_filter_cfg.id & s_filter_cfg.mask);

        return (masked_id == masked_filter);
    }

    static int find_sub_index(uint16_t id) // find index in the subscription table for an ID
    {
        for (int i = 0; i < MAX_SUBS; i++)
        {
            if (g_subs[i].in_use && g_subs[i].id == id)
            {
                return i;
            }
        }
        return -1;
    }

    static int find_free_slot()
    {
        for (int i = 0; i < MAX_SUBS; i++)
        {
            if (!g_subs[i].in_use)
            {
                return i;
            }
        }
        return -1;
    }

    static uint32_t now_ms()
    {
        return (uint32_t)(esp_timer_get_time() / 1000);
    }

    static bool on_rx_done_cb(twai_node_handle_t handle,
                              const twai_rx_done_event_data_t *edata,
                              void *user_ctx)
    {
        (void)edata;
        (void)user_ctx;

        uint8_t buf[8];
        twai_frame_t rx_frame = {
            .buffer = buf,
            .buffer_len = sizeof(buf),
        };

        if (twai_node_receive_from_isr(handle, &rx_frame) != ESP_OK)
        {
            return false;
        }

        // Only classic standard frames, DLC=8
        if (rx_frame.header.ide)
        {
            return false;
        }
        if (rx_frame.buffer_len != 8 || rx_frame.buffer == nullptr)
        {
            return false;
        }

        RxPkt pkt{};
        pkt.id = (uint16_t)(rx_frame.header.id & 0x7FFu);
        for (int i = 0; i < 8; i++)
            pkt.data[i] = rx_frame.buffer[i];

        BaseType_t hp_task_woken = pdFALSE;
        if (g_rx_queue)
        {
            (void)xQueueSendFromISR(g_rx_queue, &pkt, &hp_task_woken);
        }
        return (hp_task_woken == pdTRUE); // request yield if needed
    }

    static void rx_task(void *arg)
    {
        (void)arg;

        RxPkt pkt{};
        for (;;)
        {
            if (g_rx_queue == nullptr)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            if (xQueueReceive(g_rx_queue, &pkt, portMAX_DELAY) != pdTRUE)
            {
                continue;
            }

            const uint32_t ts = now_ms();

            void (*cb)(uint16_t, const uint8_t[8], uint32_t) = nullptr;

            if (g_subs_mutex && xSemaphoreTake(g_subs_mutex, portMAX_DELAY) == pdTRUE)
            {
                const int idx = find_sub_index(pkt.id);
                if (idx >= 0)
                    cb = g_subs[idx].callback;
                xSemaphoreGive(g_subs_mutex);
            }

            if (cb)
                cb(pkt.id, pkt.data, ts);
        }
    }

    void set_hardware_filters() // if you call empty function ==> TX only
    {
        // TX-only: accept none
        s_filter_cfg.id = 0xFFFFFFFFu;
        s_filter_cfg.mask = 0xFFFFFFFFu;
        s_filter_cfg.is_ext = false;
    }

    void set_hardware_filters(uint32_t id) // only accept one id
    {
        s_filter_cfg.id = (id & 0x7FFu);
        s_filter_cfg.mask = 0x7FFu;
        s_filter_cfg.is_ext = false;
    }

    void set_hardware_filters(uint32_t low, uint32_t high) // accepted range
    {
        low &= 0x7FFu;
        high &= 0x7FFu;
        if (low > high)
            std::swap(low, high);

        if (low == high)
        {
            set_hardware_filters(low); // prevents a minion from edging this case
            return;
        }

        const uint32_t diff = (low ^ high);                   // bitwise XOR
        const uint32_t msb = highest_set_bit_index_u32(diff); // find MSB of XOR'd high and low

        uint32_t block_mask = ~((1u << (msb + 1u)) - 1u); // compute mask
        block_mask &= 0x7FFu;

        s_filter_cfg.id = (low & block_mask); // note (high & block_mask) should give the same result (we're doing a coarse superset here)
        s_filter_cfg.mask = block_mask;
        s_filter_cfg.is_ext = false;
    }

    esp_err_t init(gpio_num_t rx_gpio, gpio_num_t tx_gpio)
    {
        if (g_initialised == true)
        {
            return ESP_ERR_INVALID_STATE; // dont call init twice
        }

        g_subs_mutex = xSemaphoreCreateMutex(); // create a mutex that protects g_subs[]
        if (g_subs_mutex == nullptr)
        {
            return ESP_ERR_NO_MEM;
        }

        if (rx_gpio == tx_gpio)
        {
            return ESP_ERR_INVALID_ARG; // if someone actually invokes this it may be over
        }

        for (int i = 0; i < MAX_SUBS; i++) // clear subscription list
        {
            g_subs[i].in_use = false;
            g_subs[i].id = 0;
            g_subs[i].callback = nullptr;
        }

        twai_onchip_node_config_t node_config = {
            // config node struct
            .io_cfg = {
                .tx = tx_gpio,
                .rx = rx_gpio,
                .quanta_clk_out = (gpio_num_t)-1,
                .bus_off_indicator = (gpio_num_t)-1,
            },
            .bit_timing = {
                .bitrate = BITRATE,
            },
            .fail_retry_cnt = FAIL_RETRY_CNT,
            .tx_queue_depth = TX_QUEUE_DEPTH,
        };

        esp_err_t err = twai_new_node_onchip(&node_config, &g_node); // create TWAI node
        if (err != ESP_OK)
        {
            return err;
        }

        err = twai_node_config_mask_filter(g_node, 0, &s_filter_cfg); // set RX filters

        if (err != ESP_OK)
        {
            twai_node_delete(g_node);
            g_node = nullptr;
            return err;
        }

        // Create queue before enabling node
        g_rx_queue = xQueueCreate(RX_QUEUE_DEPTH, sizeof(RxPkt));
        if (g_rx_queue == nullptr)
        {
            twai_node_delete(g_node);
            g_node = nullptr;
            return ESP_ERR_NO_MEM;
        }

        // Register callbacks
        twai_event_callbacks_t cbs = {
            .on_rx_done = on_rx_done_cb,
        };
        err = twai_node_register_event_callbacks(g_node, &cbs, nullptr);
        if (err != ESP_OK)
        {
            twai_node_delete(g_node);
            g_node = nullptr;
            return err;
        }

        err = twai_node_enable(g_node); // enable TWAI node

        if (err != ESP_OK)
        {
            twai_node_delete(g_node);
            g_node = nullptr;
            return ESP_FAIL;
        }

        BaseType_t task_ok = xTaskCreate( // create RX task
            rx_task,
            "MSM_CAN_RX",
            RX_TASK_STACK,
            nullptr,
            tskIDLE_PRIORITY + 1,
            nullptr);

        if (task_ok != pdPASS)
        {
            twai_node_disable(g_node);
            twai_node_delete(g_node);
            g_node = nullptr;
            return ESP_ERR_NO_MEM;
        }

        g_initialised = true;
        return ESP_OK;
    }

    esp_err_t send_msg(uint16_t id, const uint8_t data[8])
    {
        if (!g_initialised)
        {
            return ESP_ERR_INVALID_STATE;
        }

        if (!is_allowed_tx_id(id)) // allow ids only in a certain range
        {
            return ESP_ERR_INVALID_ARG;
        }

        uint8_t tx_buf[8];          // twai_frame_t uses a buffer pointer + length, so user data must remain valid until the driver is done
        for (int i = 0; i < 8; i++) // we avoid this molesting us by copying into a local array and then waiting for TX to complete
        {
            tx_buf[i] = data[i];
        }

        twai_frame_t frame = {};
        frame.header.id = id;
        frame.buffer = tx_buf;
        frame.buffer_len = 8;

        esp_err_t err = twai_node_transmit(g_node, &frame, TX_TIMEOUT_MS);
        if (err != ESP_OK)
        {
            return err;
        }

        err = twai_node_transmit_wait_all_done(g_node, TX_TIMEOUT_MS);
        return err;
    }

    esp_err_t subscribe(uint16_t id,
                        void (*callback)(uint16_t, const uint8_t[8], uint32_t))
    {
        if (!g_initialised) // if uninitialised return
        {
            return ESP_ERR_INVALID_STATE;
        }

        if (id > 0x7FFu) // if subscriving to an inval
        {
            return ESP_ERR_INVALID_ARG;
        }

        if (!is_allowed_rx_id(id)) // checks if hardware filters were setup properly
        {
            return ESP_ERR_INVALID_ARG;
        }

        if (g_subs_mutex == nullptr)
        {
            return ESP_ERR_INVALID_STATE;
        }

        if (xSemaphoreTake(g_subs_mutex, portMAX_DELAY) != pdTRUE) // lock the mutex
        {
            return ESP_FAIL;
        }

        const int existing = find_sub_index(id); // If ID is already subscribed to, don’t allocate a new slot instead update callback.
        if (existing >= 0)
        {
            g_subs[existing].callback = callback;
            xSemaphoreGive(g_subs_mutex);
            return ESP_OK;
        }

        const int slot = find_free_slot(); // if no free subscription slots then return an error
        if (slot < 0)
        {
            xSemaphoreGive(g_subs_mutex); // return the mutex
            return ESP_ERR_NO_MEM;
        }

        g_subs[slot].in_use = true;
        g_subs[slot].id = id;
        g_subs[slot].callback = callback;

        xSemaphoreGive(g_subs_mutex);
        return ESP_OK;
    }

    esp_err_t unsubscribe(uint16_t id)
    {
        if (!g_initialised)
        {
            return ESP_ERR_INVALID_STATE;
        }

        if (id > 0x7FFu)
        {
            return ESP_ERR_INVALID_ARG;
        }

        if (g_subs_mutex == nullptr)
        {
            return ESP_ERR_INVALID_STATE;
        }

        if (xSemaphoreTake(g_subs_mutex, portMAX_DELAY) != pdTRUE) // lock mutex
        {
            return ESP_FAIL;
        }

        const int idx = find_sub_index(id); // why unsubscribe to something u havent subscribed to
        if (idx < 0)
        {
            xSemaphoreGive(g_subs_mutex);
            return ESP_ERR_INVALID_STATE;
        }

        g_subs[idx].in_use = false;
        g_subs[idx].id = 0;
        g_subs[idx].callback = nullptr;

        xSemaphoreGive(g_subs_mutex); // unlock
        return ESP_OK;
    }

}