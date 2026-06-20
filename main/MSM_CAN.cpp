#include "MSM_CAN.hpp"

#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <algorithm>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

namespace MSM_CAN
{

    struct RxQueueItem
    {
        uint16_t id;
        uint8_t data[8];
    };

    struct SubEntry
    {
        bool in_use;
        bool has_last_frame;
        uint16_t id;
        RxFrame last_frame;
        RxCallback callback;
    };

    struct ScheduledEntry
    {
        bool in_use;
        TxFrame frame;
        uint32_t period_ms;
        uint32_t next_due_ms;
    };

    enum class TxCmdType : uint8_t
    {
        SendNow,
        Schedule,
        Unschedule,
        UpdatePayload,
    };

    struct TxCmd
    {
        TxCmdType type;
        TxFrame frame;
        uint32_t period_ms;

        // Public APIs wait on this so they can return the final result
        // after the TX task has processed the request.
        SemaphoreHandle_t done_sem;
        esp_err_t *result_ptr;
    };

    static constexpr size_t RX_QUEUE_DEPTH = 32;
    static constexpr size_t TX_CMD_QUEUE_DEPTH = 16;

    static constexpr uint32_t TX_TIMEOUT_MS = 10;
    static constexpr uint32_t TX_CMD_QUEUE_WAIT_MS = 50;
    static constexpr uint32_t BITRATE = 1000000;
    static constexpr uint8_t TX_QUEUE_DEPTH = 8;
    static constexpr uint8_t FAIL_RETRY_CNT = 3;

    static constexpr uint16_t RX_TASK_STACK = 4096;
    static constexpr uint16_t TX_TASK_STACK = 4096;

    static bool g_initialised = false;

    static QueueHandle_t g_rx_queue = nullptr;
    static QueueHandle_t g_tx_cmd_queue = nullptr;

    static SemaphoreHandle_t g_subs_mutex = nullptr;
    static SemaphoreHandle_t g_sched_mutex = nullptr;

    static TaskHandle_t g_rx_task_handle = nullptr;
    static TaskHandle_t g_tx_task_handle = nullptr;

    static SubEntry g_subs[MSM_CAN_MAX_SUBS];
    static ScheduledEntry g_sched[MSM_CAN_MAX_SCHEDULED_TX];

    static twai_node_handle_t g_node = nullptr;
    static bool g_node_enabled = false;

    static portMUX_TYPE g_diag_mux = portMUX_INITIALIZER_UNLOCKED;
    static Diagnostics g_diag = {};

    static twai_mask_filter_config_t s_filter_cfg =
        {
            .id = 0xFFFFFFFFu,
            .mask = 0xFFFFFFFFu,
            .is_ext = false,
    };

    static uint32_t highest_set_bit_index_u32(uint32_t x)
    {
        uint32_t idx = 0;
        while (x >>= 1)
        {
            idx++;
        }
        return idx;
    }

    static void diag_reset()
    {
        portENTER_CRITICAL(&g_diag_mux);
        g_diag.rx_drops = 0;
        g_diag.ignored_frames = 0;
        g_diag.tx_failures = 0;
        g_diag.scheduled_sends = 0;
        g_diag.last_tx_error = ESP_OK;
        g_diag.last_rx_error = ESP_OK;
        portEXIT_CRITICAL(&g_diag_mux);
    }

    static void diag_note_rx_drop_from_isr(esp_err_t err)
    {
        portENTER_CRITICAL_ISR(&g_diag_mux);
        g_diag.rx_drops++;
        g_diag.last_rx_error = err;
        portEXIT_CRITICAL_ISR(&g_diag_mux);
    }

    static void diag_note_rx_drop(esp_err_t err)
    {
        portENTER_CRITICAL(&g_diag_mux);
        g_diag.rx_drops++;
        g_diag.last_rx_error = err;
        portEXIT_CRITICAL(&g_diag_mux);
    }

    static void diag_note_ignored_frame_from_isr(esp_err_t err)
    {
        portENTER_CRITICAL_ISR(&g_diag_mux);
        g_diag.ignored_frames++;
        g_diag.last_rx_error = err;
        portEXIT_CRITICAL_ISR(&g_diag_mux);
    }

    static void diag_note_ignored_frame(esp_err_t err)
    {
        portENTER_CRITICAL(&g_diag_mux);
        g_diag.ignored_frames++;
        g_diag.last_rx_error = err;
        portEXIT_CRITICAL(&g_diag_mux);
    }

    static void diag_note_tx_failure(esp_err_t err)
    {
        portENTER_CRITICAL(&g_diag_mux);
        g_diag.tx_failures++;
        g_diag.last_tx_error = err;
        portEXIT_CRITICAL(&g_diag_mux);
    }

    static void diag_note_scheduled_send()
    {
        portENTER_CRITICAL(&g_diag_mux);
        g_diag.scheduled_sends++;
        portEXIT_CRITICAL(&g_diag_mux);
    }

    static void diag_note_tx_result(esp_err_t err)
    {
        if (err != ESP_OK)
        {
            diag_note_tx_failure(err);
        }
    }

    static void diag_note_scheduled_tx_result(esp_err_t err)
    {
        if (err == ESP_OK)
        {
            diag_note_scheduled_send();
        }
        else
        {
            diag_note_tx_failure(err);
        }
    }

    static void clear_subscriptions()
    {
        for (int i = 0; i < MSM_CAN_MAX_SUBS; i++)
        {
            g_subs[i].in_use = false;
            g_subs[i].has_last_frame = false;
            g_subs[i].id = 0;
            g_subs[i].last_frame.id = 0;
            clear_payload(g_subs[i].last_frame.data);
            g_subs[i].last_frame.timestamp_ms = 0;
            g_subs[i].callback = nullptr;
        }
    }

    static void clear_scheduled_entries()
    {
        for (int i = 0; i < MSM_CAN_MAX_SCHEDULED_TX; i++)
        {
            g_sched[i].in_use = false;
            g_sched[i].frame.id = 0;
            g_sched[i].period_ms = 0;
            g_sched[i].next_due_ms = 0;
            clear_payload(g_sched[i].frame.data);
        }
    }

    static void cleanup_init_resources()
    {
        g_initialised = false;

        if (g_rx_task_handle != nullptr)
        {
            vTaskDelete(g_rx_task_handle);
            g_rx_task_handle = nullptr;
        }

        if (g_tx_task_handle != nullptr)
        {
            vTaskDelete(g_tx_task_handle);
            g_tx_task_handle = nullptr;
        }

        if (g_node != nullptr)
        {
            if (g_node_enabled)
            {
                (void)twai_node_disable(g_node);
                g_node_enabled = false;
            }

            twai_node_delete(g_node);
            g_node = nullptr;
        }
        g_node_enabled = false;

        if (g_tx_cmd_queue != nullptr)
        {
            vQueueDelete(g_tx_cmd_queue);
            g_tx_cmd_queue = nullptr;
        }

        if (g_rx_queue != nullptr)
        {
            vQueueDelete(g_rx_queue);
            g_rx_queue = nullptr;
        }

        if (g_sched_mutex != nullptr)
        {
            vSemaphoreDelete(g_sched_mutex);
            g_sched_mutex = nullptr;
        }

        if (g_subs_mutex != nullptr)
        {
            vSemaphoreDelete(g_subs_mutex);
            g_subs_mutex = nullptr;
        }

        clear_subscriptions();
        clear_scheduled_entries();
    }

    static bool is_allowed_rx_id(uint16_t id)
    {
        const uint32_t masked_id = (id & s_filter_cfg.mask);
        const uint32_t masked_filter = (s_filter_cfg.id & s_filter_cfg.mask);

        return (masked_id == masked_filter);
    }

    static int find_sub_index(uint16_t id)
    {
        for (int i = 0; i < MSM_CAN_MAX_SUBS; i++)
        {
            if (g_subs[i].in_use && g_subs[i].id == id)
            {
                return i;
            }
        }
        return -1;
    }

    static int find_free_sub_slot()
    {
        for (int i = 0; i < MSM_CAN_MAX_SUBS; i++)
        {
            if (!g_subs[i].in_use)
            {
                return i;
            }
        }
        return -1;
    }

    static int find_sched_index(uint16_t id)
    {
        for (int i = 0; i < MSM_CAN_MAX_SCHEDULED_TX; i++)
        {
            if (g_sched[i].in_use && g_sched[i].frame.id == id)
            {
                return i;
            }
        }
        return -1;
    }

    static int find_free_sched_slot()
    {
        for (int i = 0; i < MSM_CAN_MAX_SCHEDULED_TX; i++)
        {
            if (!g_sched[i].in_use)
            {
                return i;
            }
        }
        return -1;
    }

    static uint32_t now_ms()
    {
        return static_cast<uint32_t>(esp_timer_get_time() / 1000);
    }

    static bool time_reached(uint32_t now, uint32_t deadline)
    {
        return static_cast<int32_t>(now - deadline) >= 0;
    }

    static void copy_payload(uint8_t dst[8], const uint8_t src[8])
    {
        for (int i = 0; i < 8; i++)
        {
            dst[i] = src[i];
        }
    }

    static void copy_tx_frame(TxFrame& dst, const TxFrame& src)
    {
        dst.id = src.id;
        copy_payload(dst.data, src.data);
    }

    static void copy_rx_frame(RxFrame& dst, const RxFrame& src)
    {
        dst.id = src.id;
        copy_payload(dst.data, src.data);
        dst.timestamp_ms = src.timestamp_ms;
    }

    static esp_err_t transmit_frame_blocking(const TxFrame& tx_frame)
    {
        if (g_node == nullptr)
        {
            return ESP_ERR_INVALID_STATE;
        }

        uint8_t tx_buf[8];
        copy_payload(tx_buf, tx_frame.data);

        twai_frame_t frame = {};
        frame.header.id = tx_frame.id;
        frame.buffer = tx_buf;
        frame.buffer_len = 8;

        esp_err_t err = twai_node_transmit(g_node, &frame, TX_TIMEOUT_MS);
        if (err != ESP_OK)
        {
            return err;
        }

        return twai_node_transmit_wait_all_done(g_node, TX_TIMEOUT_MS);
    }

    static void complete_tx_cmd(TxCmd &cmd, esp_err_t result)
    {
        if (cmd.result_ptr != nullptr)
        {
            *(cmd.result_ptr) = result;
        }

        if (cmd.done_sem != nullptr)
        {
            xSemaphoreGive(cmd.done_sem);
        }
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

        const esp_err_t err = twai_node_receive_from_isr(handle, &rx_frame);
        if (err != ESP_OK)
        {
            diag_note_rx_drop_from_isr(err);
            return false;
        }

        if (rx_frame.header.ide)
        {
            diag_note_ignored_frame_from_isr(ESP_ERR_INVALID_ARG);
            return false;
        }
        if (rx_frame.buffer_len != 8 || rx_frame.buffer == nullptr)
        {
            diag_note_ignored_frame_from_isr(ESP_ERR_INVALID_SIZE);
            return false;
        }

        RxQueueItem pkt{};
        pkt.id = static_cast<uint16_t>(rx_frame.header.id & 0x7FFu);
        for (int i = 0; i < 8; i++)
        {
            pkt.data[i] = rx_frame.buffer[i];
        }

        BaseType_t hp_task_woken = pdFALSE;
        if (g_rx_queue != nullptr)
        {
            if (xQueueSendFromISR(g_rx_queue, &pkt, &hp_task_woken) != pdTRUE)
            {
                diag_note_rx_drop_from_isr(ESP_ERR_TIMEOUT);
            }
        }
        else
        {
            diag_note_rx_drop_from_isr(ESP_ERR_INVALID_STATE);
        }

        return (hp_task_woken == pdTRUE);
    }

    static void rx_task(void *arg)
    {
        (void)arg;

        RxQueueItem pkt{};
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

            RxFrame frame{};
            frame.id = pkt.id;
            copy_payload(frame.data, pkt.data);
            frame.timestamp_ms = now_ms();

            RxCallback cb = nullptr;

            if (g_subs_mutex != nullptr &&
                xSemaphoreTake(g_subs_mutex, portMAX_DELAY) == pdTRUE)
            {
                const int idx = find_sub_index(pkt.id);
                if (idx >= 0)
                {
                    copy_rx_frame(g_subs[idx].last_frame, frame);
                    g_subs[idx].has_last_frame = true;
                    cb = g_subs[idx].callback;
                }
                else
                {
                    diag_note_ignored_frame(ESP_ERR_NOT_FOUND);
                }
                xSemaphoreGive(g_subs_mutex);
            }
            else
            {
                diag_note_rx_drop(ESP_ERR_INVALID_STATE);
            }

            if (cb != nullptr)
            {
                cb(frame);
            }
        }
    }

    static uint32_t compute_next_tx_wait_ms()
    {
        if (g_sched_mutex == nullptr)
        {
            return portMAX_DELAY;
        }

        uint32_t min_wait = UINT32_MAX;
        bool have_scheduled = false;
        const uint32_t now = now_ms();

        if (xSemaphoreTake(g_sched_mutex, portMAX_DELAY) != pdTRUE)
        {
            return 10;
        }

        for (int i = 0; i < MSM_CAN_MAX_SCHEDULED_TX; i++)
        {
            if (!g_sched[i].in_use)
            {
                continue;
            }

            have_scheduled = true;

            if (time_reached(now, g_sched[i].next_due_ms))
            {
                min_wait = 0;
                break;
            }

            const uint32_t wait_ms = g_sched[i].next_due_ms - now;
            if (wait_ms < min_wait)
            {
                min_wait = wait_ms;
            }
        }

        xSemaphoreGive(g_sched_mutex);

        if (!have_scheduled)
        {
            return portMAX_DELAY;
        }

        return min_wait;
    }

    static void process_due_scheduled_messages()
    {
        if (g_sched_mutex == nullptr)
        {
            return;
        }

        const uint32_t now = now_ms();

        for (int i = 0; i < MSM_CAN_MAX_SCHEDULED_TX; i++)
        {
            TxFrame frame{};
            bool should_send = false;

            if (xSemaphoreTake(g_sched_mutex, portMAX_DELAY) != pdTRUE)
            {
                return;
            }

            if (g_sched[i].in_use && time_reached(now, g_sched[i].next_due_ms))
            {
                copy_tx_frame(frame, g_sched[i].frame);

                // Advance by one period from now so delayed sends do not
                // generate a burst of catch-up traffic.
                g_sched[i].next_due_ms = now + g_sched[i].period_ms;
                should_send = true;
            }

            xSemaphoreGive(g_sched_mutex);

            if (should_send)
            {
                const esp_err_t err = transmit_frame_blocking(frame);
                diag_note_scheduled_tx_result(err);
            }
        }
    }

    static void handle_tx_cmd(TxCmd &cmd)
    {
        esp_err_t result = ESP_OK;

        switch (cmd.type)
        {
        case TxCmdType::SendNow:
        {
            result = transmit_frame_blocking(cmd.frame);
            diag_note_tx_result(result);
            break;
        }

        case TxCmdType::Schedule:
        {
            if (g_sched_mutex == nullptr)
            {
                result = ESP_ERR_INVALID_STATE;
                break;
            }

            if (xSemaphoreTake(g_sched_mutex, portMAX_DELAY) != pdTRUE)
            {
                result = ESP_FAIL;
                break;
            }

            const int existing = find_sched_index(cmd.frame.id);
            if (existing >= 0)
            {
                copy_tx_frame(g_sched[existing].frame, cmd.frame);
                g_sched[existing].period_ms = cmd.period_ms;
                g_sched[existing].next_due_ms = now_ms() + cmd.period_ms;
                result = ESP_OK;
            }
            else
            {
                const int slot = find_free_sched_slot();
                if (slot < 0)
                {
                    result = ESP_ERR_NO_MEM;
                }
                else
                {
                    g_sched[slot].in_use = true;
                    copy_tx_frame(g_sched[slot].frame, cmd.frame);
                    g_sched[slot].period_ms = cmd.period_ms;
                    g_sched[slot].next_due_ms = now_ms() + cmd.period_ms;
                    result = ESP_OK;
                }
            }

            xSemaphoreGive(g_sched_mutex);
            break;
        }

        case TxCmdType::Unschedule:
        {
            if (g_sched_mutex == nullptr)
            {
                result = ESP_ERR_INVALID_STATE;
                break;
            }

            if (xSemaphoreTake(g_sched_mutex, portMAX_DELAY) != pdTRUE)
            {
                result = ESP_FAIL;
                break;
            }

            const int idx = find_sched_index(cmd.frame.id);
            if (idx < 0)
            {
                result = ESP_ERR_INVALID_STATE;
            }
            else
            {
                g_sched[idx].in_use = false;
                g_sched[idx].frame.id = 0;
                g_sched[idx].period_ms = 0;
                g_sched[idx].next_due_ms = 0;
                clear_payload(g_sched[idx].frame.data);
                result = ESP_OK;
            }

            xSemaphoreGive(g_sched_mutex);
            break;
        }

        case TxCmdType::UpdatePayload:
        {
            if (g_sched_mutex == nullptr)
            {
                result = ESP_ERR_INVALID_STATE;
                break;
            }

            if (xSemaphoreTake(g_sched_mutex, portMAX_DELAY) != pdTRUE)
            {
                result = ESP_FAIL;
                break;
            }

            const int idx = find_sched_index(cmd.frame.id);
            if (idx < 0)
            {
                result = ESP_ERR_INVALID_STATE;
            }
            else
            {
                // Update only the stored payload. The schedule phase is kept.
                copy_payload(g_sched[idx].frame.data, cmd.frame.data);
                result = ESP_OK;
            }

            xSemaphoreGive(g_sched_mutex);
            break;
        }

        default:
        {
            result = ESP_ERR_INVALID_ARG;
            break;
        }
        }

        complete_tx_cmd(cmd, result);
    }

    static void tx_task(void *arg)
    {
        (void)arg;

        TxCmd cmd{};

        for (;;)
        {
            if (g_tx_cmd_queue == nullptr)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            const uint32_t wait_ms = compute_next_tx_wait_ms();
            const TickType_t wait_ticks =
                (wait_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(wait_ms);

            if (xQueueReceive(g_tx_cmd_queue, &cmd, wait_ticks) == pdTRUE)
            {
                handle_tx_cmd(cmd);
            }

            process_due_scheduled_messages();
        }
    }

    esp_err_t set_hardware_filters()
    {
        if (g_initialised)
        {
            return ESP_ERR_INVALID_STATE;
        }

        s_filter_cfg.id = 0xFFFFFFFFu;
        s_filter_cfg.mask = 0xFFFFFFFFu;
        s_filter_cfg.is_ext = false;
        return ESP_OK;
    }

    esp_err_t set_hardware_filters(uint32_t id)
    {
        if (g_initialised)
        {
            return ESP_ERR_INVALID_STATE;
        }

        s_filter_cfg.id = (id & 0x7FFu);
        s_filter_cfg.mask = 0x7FFu;
        s_filter_cfg.is_ext = false;
        return ESP_OK;
    }

    esp_err_t set_hardware_filters(uint32_t low, uint32_t high)
    {
        if (g_initialised)
        {
            return ESP_ERR_INVALID_STATE;
        }

        low &= 0x7FFu;
        high &= 0x7FFu;
        if (low > high)
        {
            std::swap(low, high);
        }

        if (low == high)
        {
            return set_hardware_filters(low);
        }

        const uint32_t diff = (low ^ high);
        const uint32_t msb = highest_set_bit_index_u32(diff);

        uint32_t block_mask = ~((1u << (msb + 1u)) - 1u);
        block_mask &= 0x7FFu;

        s_filter_cfg.id = (low & block_mask);
        s_filter_cfg.mask = block_mask;
        s_filter_cfg.is_ext = false;
        return ESP_OK;
    }

    esp_err_t init(gpio_num_t rx_gpio, gpio_num_t tx_gpio)
    {
        if (g_initialised)
        {
            return ESP_ERR_INVALID_STATE;
        }

        if (rx_gpio == tx_gpio)
        {
            return ESP_ERR_INVALID_ARG;
        }

        diag_reset();

        const BaseType_t task_core_id = xPortGetCoreID();

        g_subs_mutex = xSemaphoreCreateMutex();
        if (g_subs_mutex == nullptr)
        {
            cleanup_init_resources();
            return ESP_ERR_NO_MEM;
        }

        g_sched_mutex = xSemaphoreCreateMutex();
        if (g_sched_mutex == nullptr)
        {
            cleanup_init_resources();
            return ESP_ERR_NO_MEM;
        }

        clear_subscriptions();
        clear_scheduled_entries();

        twai_onchip_node_config_t node_config = {
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

        esp_err_t err = twai_new_node_onchip(&node_config, &g_node);
        if (err != ESP_OK)
        {
            cleanup_init_resources();
            return err;
        }

        err = twai_node_config_mask_filter(g_node, 0, &s_filter_cfg);
        if (err != ESP_OK)
        {
            cleanup_init_resources();
            return err;
        }

        g_rx_queue = xQueueCreate(RX_QUEUE_DEPTH, sizeof(RxQueueItem));
        if (g_rx_queue == nullptr)
        {
            cleanup_init_resources();
            return ESP_ERR_NO_MEM;
        }

        g_tx_cmd_queue = xQueueCreate(TX_CMD_QUEUE_DEPTH, sizeof(TxCmd));
        if (g_tx_cmd_queue == nullptr)
        {
            cleanup_init_resources();
            return ESP_ERR_NO_MEM;
        }

        twai_event_callbacks_t cbs = {
            .on_rx_done = on_rx_done_cb,
        };
        err = twai_node_register_event_callbacks(g_node, &cbs, nullptr);
        if (err != ESP_OK)
        {
            cleanup_init_resources();
            return err;
        }

        err = twai_node_enable(g_node);
        if (err != ESP_OK)
        {
            cleanup_init_resources();
            return ESP_FAIL;
        }
        g_node_enabled = true;

        BaseType_t rx_task_ok = xTaskCreatePinnedToCore(
            rx_task,
            "MSM_CAN_RX",
            RX_TASK_STACK,
            nullptr,
            tskIDLE_PRIORITY + 1,
            &g_rx_task_handle,
            task_core_id);

        if (rx_task_ok != pdPASS)
        {
            g_rx_task_handle = nullptr;
            cleanup_init_resources();
            return ESP_ERR_NO_MEM;
        }

        BaseType_t tx_task_ok = xTaskCreatePinnedToCore(
            tx_task,
            "MSM_CAN_TX",
            TX_TASK_STACK,
            nullptr,
            tskIDLE_PRIORITY + 1,
            &g_tx_task_handle,
            task_core_id);

        if (tx_task_ok != pdPASS)
        {
            g_tx_task_handle = nullptr;
            cleanup_init_resources();
            return ESP_ERR_NO_MEM;
        }

        g_initialised = true;
        return ESP_OK;
    }

    static esp_err_t submit_tx_cmd_and_wait(TxCmdType type,
                                            const TxFrame& frame,
                                            uint32_t period_ms)
    {
        if (!g_initialised)
        {
            return ESP_ERR_INVALID_STATE;
        }

        if (frame.id > 0x7FFu)
        {
            return ESP_ERR_INVALID_ARG;
        }

        if (type == TxCmdType::Schedule && period_ms == 0)
        {
            return ESP_ERR_INVALID_ARG;
        }

        if (g_tx_cmd_queue == nullptr)
        {
            return ESP_ERR_INVALID_STATE;
        }

        SemaphoreHandle_t done_sem = xSemaphoreCreateBinary();
        if (done_sem == nullptr)
        {
            return ESP_ERR_NO_MEM;
        }

        esp_err_t result = ESP_FAIL;

        TxCmd cmd{};
        cmd.type = type;
        copy_tx_frame(cmd.frame, frame);
        cmd.period_ms = period_ms;
        cmd.done_sem = done_sem;
        cmd.result_ptr = &result;

        if (xQueueSend(g_tx_cmd_queue, &cmd, pdMS_TO_TICKS(TX_CMD_QUEUE_WAIT_MS)) != pdTRUE)
        {
            vSemaphoreDelete(done_sem);
            diag_note_tx_failure(ESP_ERR_TIMEOUT);
            return ESP_ERR_TIMEOUT;
        }

        (void)xSemaphoreTake(done_sem, portMAX_DELAY);

        vSemaphoreDelete(done_sem);
        return result;
    }

    esp_err_t send_msg(const TxFrame& frame)
    {
        return submit_tx_cmd_and_wait(TxCmdType::SendNow, frame, 0);
    }

    esp_err_t schedule(const TxFrame& frame, uint32_t period_ms)
    {
        return submit_tx_cmd_and_wait(TxCmdType::Schedule, frame, period_ms);
    }

    esp_err_t update_scheduled_payload(const TxFrame& frame)
    {
        return submit_tx_cmd_and_wait(TxCmdType::UpdatePayload, frame, 0);
    }

    esp_err_t unschedule(uint16_t id)
    {
        TxFrame frame{};
        frame.id = id;
        clear_payload(frame.data);
        return submit_tx_cmd_and_wait(TxCmdType::Unschedule, frame, 0);
    }

    esp_err_t subscribe(uint16_t id, RxCallback callback)
    {
        if (!g_initialised)
        {
            return ESP_ERR_INVALID_STATE;
        }

        if (id > 0x7FFu)
        {
            return ESP_ERR_INVALID_ARG;
        }

        if (!is_allowed_rx_id(id))
        {
            return ESP_ERR_INVALID_ARG;
        }

        if (g_subs_mutex == nullptr)
        {
            return ESP_ERR_INVALID_STATE;
        }

        if (xSemaphoreTake(g_subs_mutex, portMAX_DELAY) != pdTRUE)
        {
            return ESP_FAIL;
        }

        const int existing = find_sub_index(id);
        if (existing >= 0)
        {
            g_subs[existing].callback = callback;
            xSemaphoreGive(g_subs_mutex);
            return ESP_OK;
        }

        const int slot = find_free_sub_slot();
        if (slot < 0)
        {
            xSemaphoreGive(g_subs_mutex);
            return ESP_ERR_NO_MEM;
        }

        g_subs[slot].in_use = true;
        g_subs[slot].has_last_frame = false;
        g_subs[slot].id = id;
        g_subs[slot].last_frame.id = 0;
        clear_payload(g_subs[slot].last_frame.data);
        g_subs[slot].last_frame.timestamp_ms = 0;
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

        if (xSemaphoreTake(g_subs_mutex, portMAX_DELAY) != pdTRUE)
        {
            return ESP_FAIL;
        }

        const int idx = find_sub_index(id);
        if (idx < 0)
        {
            xSemaphoreGive(g_subs_mutex);
            return ESP_ERR_INVALID_STATE;
        }

        g_subs[idx].in_use = false;
        g_subs[idx].has_last_frame = false;
        g_subs[idx].id = 0;
        g_subs[idx].last_frame.id = 0;
        clear_payload(g_subs[idx].last_frame.data);
        g_subs[idx].last_frame.timestamp_ms = 0;
        g_subs[idx].callback = nullptr;

        xSemaphoreGive(g_subs_mutex);
        return ESP_OK;
    }

    esp_err_t get(uint16_t id, RxFrame& frame)
    {
        frame.id = 0;
        clear_payload(frame.data);
        frame.timestamp_ms = 0;

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

        if (xSemaphoreTake(g_subs_mutex, portMAX_DELAY) != pdTRUE)
        {
            return ESP_FAIL;
        }

        const int idx = find_sub_index(id);
        if (idx < 0 || !g_subs[idx].has_last_frame)
        {
            xSemaphoreGive(g_subs_mutex);
            return ESP_ERR_NOT_FOUND;
        }

        copy_rx_frame(frame, g_subs[idx].last_frame);
                
        xSemaphoreGive(g_subs_mutex);
        return ESP_OK;
    }

    esp_err_t get_and_clear(uint16_t id, RxFrame& frame)
    {
        frame.id = 0;
        clear_payload(frame.data);
        frame.timestamp_ms = 0;

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

        if (xSemaphoreTake(g_subs_mutex, portMAX_DELAY) != pdTRUE)
        {
            return ESP_FAIL;
        }

        const int idx = find_sub_index(id);
        if (idx < 0 || !g_subs[idx].has_last_frame)
        {
            xSemaphoreGive(g_subs_mutex);
            return ESP_ERR_NOT_FOUND;
        }

        copy_rx_frame(frame, g_subs[idx].last_frame);
        g_subs[idx].has_last_frame = false;
        g_subs[idx].last_frame.id = 0;
        clear_payload(g_subs[idx].last_frame.data);
        g_subs[idx].last_frame.timestamp_ms = 0;

        xSemaphoreGive(g_subs_mutex);
        return ESP_OK;
    }

    void get_diagnostics(Diagnostics& diagnostics)
    {
        portENTER_CRITICAL(&g_diag_mux);
        diagnostics = g_diag;
        portEXIT_CRITICAL(&g_diag_mux);
    }

    void reset_diagnostics()
    {
        diag_reset();
    }

}
