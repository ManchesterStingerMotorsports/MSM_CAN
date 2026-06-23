#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_err.h"
#include "driver/gpio.h" 

#ifndef MSM_CAN_MAX_SUBS
#define MSM_CAN_MAX_SUBS 64
#endif

#ifndef MSM_CAN_MAX_SCHEDULED_TX
#define MSM_CAN_MAX_SCHEDULED_TX 32
#endif

namespace MSM_CAN
{
    struct TxFrame
    {
        uint16_t id;
        uint8_t data[8];
    };

    struct RxFrame
    {
        uint16_t id;
        uint8_t data[8];
        // Receive ISR time in milliseconds since boot, wrapping at 32 bits.
        uint32_t timestamp_ms;
    };

    struct Diagnostics
    {
        uint32_t rx_drops;
        uint32_t ignored_frames;
        uint32_t tx_failures;
        uint32_t scheduled_sends;
        esp_err_t last_tx_error;
        esp_err_t last_rx_error;
        uint32_t bus_off_events;
        uint32_t bus_errors;
    };

    enum class BusState : uint8_t { Unknown, Active, Warning, Passive, BusOff };

    struct BusStatus
    {
        BusState state;
        uint16_t tx_error_count;
        uint16_t rx_error_count;
    };

    using RxCallback = void (*)(const RxFrame& frame);

    esp_err_t init(gpio_num_t rx_gpio, gpio_num_t tx_gpio);  

    // Queue a one-shot transmit and wait for the TX task to report the result.
    // Bounded waits: 50 ms queue admission + 100 ms completion (plus task scheduling).
    // A timeout does not cancel a transmission already submitted to TWAI.
    esp_err_t send_msg(const TxFrame& frame);

    esp_err_t get_bus_status(BusStatus& status);
    // Start asynchronous bus-off recovery; poll get_bus_status() for completion.
    // Pending driver traffic may resume; scheduled entries are retained.
    esp_err_t recover();

   
    // Schedule a frame to be sent periodically by the background TX task.
    // Re-scheduling the same ID updates its payload/period and restarts its timing.
    esp_err_t schedule(const TxFrame& frame, uint32_t period_ms);
    
    // Update only the stored payload for a scheduled ID.
    // The existing schedule phase and period are left unchanged.
    esp_err_t update_scheduled_payload(const TxFrame& frame);
    
    // Remove a scheduled transmit entry for the given ID.
    esp_err_t unschedule(uint16_t id);


    esp_err_t subscribe(uint16_t id, RxCallback callback = nullptr);
    esp_err_t unsubscribe(uint16_t id);
    
    esp_err_t get(uint16_t id, RxFrame& frame);
    esp_err_t get_and_clear(uint16_t id, RxFrame& frame);

    void get_diagnostics(Diagnostics& diagnostics);
    void reset_diagnostics();

    esp_err_t set_hardware_filters();
    esp_err_t set_hardware_filters(uint32_t id);
    esp_err_t set_hardware_filters(uint32_t low, uint32_t high);
       
    inline void pack_u16(uint8_t data[8], uint8_t index, uint16_t value)                //helper function to pack uint8_t data[8] with a big-endian encoded uint16_t 
    {
        if (data == nullptr || index > 6) return;

        data[index + 0] = static_cast<uint8_t>((value >> 8) & 0xFF);
        data[index + 1] = static_cast<uint8_t>((value >> 0) & 0xFF);
    }

    inline void pack_i16(uint8_t data[8], uint8_t index, int16_t value)                 //helper function to pack uint8_t data[8] with a big-endian encoded int16_t
    {
        pack_u16(data, index, static_cast<uint16_t>(value));
    }


    inline void pack_u32(uint8_t data[8], uint8_t index, uint32_t value)                //helper function to pack uint8_t data[8] with a big-endian encoded uint32_t 
    {
        if (data == nullptr || index > 4) return;

        data[index + 0] = static_cast<uint8_t>((value >> 24) & 0xFF);
        data[index + 1] = static_cast<uint8_t>((value >> 16) & 0xFF);
        data[index + 2] = static_cast<uint8_t>((value >> 8)  & 0xFF);
        data[index + 3] = static_cast<uint8_t>((value >> 0)  & 0xFF);
    }

    inline void pack_u8(uint8_t data[8], uint8_t index, uint8_t value)                  // helper function to pack uint8_t data[8] with a uint8_t
    {
        if (data == nullptr || index > 7) return;

        data[index] = value;
    }

    inline void pack_i8(uint8_t data[8], uint8_t index, int8_t value)                   //helper function to pack uint8_t data[8] with an int8_t
    {
        pack_u8(data, index, static_cast<uint8_t>(value));
    }

    inline void pack_float(uint8_t data[8], uint8_t index, float value)                  // helper function to pack uint8_t data[8] with a big-endian encoded float
    {
        uint32_t bits = 0;
        memcpy(&bits, &value, sizeof(bits));
        pack_u32(data, index, bits);
    }

    inline uint8_t unpack_u8(const uint8_t data[8], uint8_t index)                  // helper function to unpack uint8_t data[8] with a uint8_t
    {
        if (data == nullptr || index > 7) return 0;

        return data[index];
    }

    inline int8_t unpack_i8(const uint8_t data[8], uint8_t index)                   // helper function to unpack uint8_t data[8] with an int8_t
    {
        return static_cast<int8_t>(unpack_u8(data, index));
    }

    inline uint16_t unpack_u16(const uint8_t data[8], uint8_t index)                // helper function to unpack uint8_t data[8] with a big-endian encoded uint16_t
    {
        if (data == nullptr || index > 6) return 0;

        return static_cast<uint16_t>(
            (static_cast<uint16_t>(data[index + 0]) << 8) |
            (static_cast<uint16_t>(data[index + 1]) << 0));                         // the <<0 is just for stylistic consistency
    }

    inline int16_t unpack_i16(const uint8_t data[8], uint8_t index)                 // helper function to unpack uint8_t data[8] with a big-endian encoded int16_t
    {
        return static_cast<int16_t>(unpack_u16(data, index));
    }

    inline uint32_t unpack_u32(const uint8_t data[8], uint8_t index)                // helper function to unpack uint8_t data[8] with a big-endian encoded uint32_t
    {
        if (data == nullptr || index > 4) return 0;

        return (static_cast<uint32_t>(data[index + 0]) << 24) |
               (static_cast<uint32_t>(data[index + 1]) << 16) |
               (static_cast<uint32_t>(data[index + 2]) << 8)  |
               (static_cast<uint32_t>(data[index + 3]) << 0);
    }

    inline float unpack_float(const uint8_t data[8], uint8_t index)                 // helper function to unpack uint8_t data[8] with a big-endian encoded float
    {
        const uint32_t bits = unpack_u32(data, index);
        float value = 0.0f;
        memcpy(&value, &bits, sizeof(value));                                        // safer than using pointer casts
        return value;
    }

    inline bool check_flag(const uint8_t data[8], uint8_t byte, uint8_t bit)            // helper function to check flag bits in payload data (bit 0 is LSB)
    {
        if (data == nullptr || byte > 7 || bit > 7) return false;

        return ((data[byte] >> bit) & 0x1u) != 0u;
    }
    

    inline void set_bit(uint8_t& byte, uint8_t bit_position, bool value)               //helper function that allows direct bit manipulation (useful for flags)
    {
        if (bit_position > 7) return;

        const uint8_t mask = static_cast<uint8_t>(1u << bit_position);

        if (value) byte |= mask;
        else       byte &= static_cast<uint8_t>(~mask);
    }

  
    inline void clear_payload(uint8_t data[8])                                         //clears the data 
    {
        if (data == nullptr) return;

        for (int i = 0; i < 8; i++) data[i] = 0;
    }

}

