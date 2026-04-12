#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_err.h"
#include "driver/gpio.h" 

namespace MSM_CAN
{
    esp_err_t init(gpio_num_t rx_gpio, gpio_num_t tx_gpio);  

    // Queue a one-shot transmit and wait for the TX task to report the result.
    // This remains a blocking API from the caller's point of view.
    esp_err_t send_msg(uint16_t id, const uint8_t data[8]);

   
    // Schedule a frame to be sent periodically by the background TX task.
    // Re-scheduling the same ID updates its payload/period and restarts its timing.
    esp_err_t schedule(uint16_t id, const uint8_t data[8], uint32_t period_ms);
    
    // Update only the stored payload for a scheduled ID.
    // The existing schedule phase and period are left unchanged.
    esp_err_t update_scheduled_payload(uint16_t id,
                                        const uint8_t data[8]);
    
    // Remove a scheduled transmit entry for the given ID.
    esp_err_t unschedule(uint16_t id);


    esp_err_t subscribe(uint16_t id,
                        void (*callback)(uint16_t id, const uint8_t data[8], uint32_t timestamp) = nullptr);
    esp_err_t unsubscribe(uint16_t id);
    esp_err_t get(uint16_t id, uint8_t data_out[8], uint32_t *timestamp_ms = nullptr);

    void set_hardware_filters();
    void set_hardware_filters(uint32_t id);
    void set_hardware_filters(uint32_t low, uint32_t high);
       
    inline void pack_u16(uint8_t data[8], uint8_t index, uint16_t value)                //helper function to pack uint8_t data[8] with a big-endian encoded uint16_t 
    {
        if (index > 6) return;

        data[index + 0] = static_cast<uint8_t>((value >> 8) & 0xFF);
        data[index + 1] = static_cast<uint8_t>((value >> 0) & 0xFF);
    }


    inline void pack_u32(uint8_t data[8], uint8_t index, uint32_t value)                //helper function to pack uint8_t data[8] with a big-endian encoded uint32_t 
    {
        if (index > 4) return;

        data[index + 0] = static_cast<uint8_t>((value >> 24) & 0xFF);
        data[index + 1] = static_cast<uint8_t>((value >> 16) & 0xFF);
        data[index + 2] = static_cast<uint8_t>((value >> 8)  & 0xFF);
        data[index + 3] = static_cast<uint8_t>((value >> 0)  & 0xFF);
    }

    inline void pack_u8(uint8_t data[8], uint8_t index, uint8_t value)                  //by all means useless but might make more sense to minions / make code slightly more legible
    {
        if (index > 7) return;

        data[index] = value;
    }

    inline void pack_float(uint8_t data[8], uint8_t index, float value)                  // helper function to pack uint8_t data[8] with a big-endian encoded float
    {
        uint32_t bits = 0;
        memcpy(&bits, &value, sizeof(bits));
        pack_u32(data, index, bits);
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
        if (byte > 7 || bit > 7) return false;

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
        for (int i = 0; i < 8; i++) data[i] = 0;
    }

}

