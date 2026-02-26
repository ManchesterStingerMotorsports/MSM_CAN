#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h" 

namespace MSM_CAN
{
    esp_err_t init(gpio_num_t rx_gpio, gpio_num_t tx_gpio);  

    esp_err_t send_msg(uint16_t id, const uint8_t data[8]);
    esp_err_t subscribe(uint16_t id,
                        void (*callback)(uint16_t id, const uint8_t data[8], uint32_t timestamp) = nullptr);
    esp_err_t unsubscribe(uint16_t id);

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

