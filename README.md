# MSM_CAN - TWAI Driver for ESP32 

Classic CAN 2.0 library for ESP32 (ESP-IDF, TWAI on-chip driver).

Designed for Manchester Stinger Motorsports by James Platt 

---

## Bus Configuration

- Bitrate: **1 Mbit/s**
- Protocol: **Classic CAN 2.0**
- Identifier: **11-bit standard IDs only**
- Payload length: **Always 8 bytes (DLC = 8)**

This library is intentionally strict.
---

# Design Philosophy

The impressive thing about this library is not how complicated it is, how clever
the internals are, or how many lines of code it contains. The point is the
opposite: it makes CAN usage simple.

> "An idiot admires complexity; A genius admires simplicity"  
> - Terry A. Davis

This library:

- Transmits and receives raw `uint8_t[8]` payloads
- Supports periodic scheduled transmission
- Enforces TX ID ranges
- Manages hardware filtering
- Provides safe callback-based reception
- Caches the latest received packet per subscribed ID
- Protects internal state with FreeRTOS mutexes
- Avoids dynamic memory allocation
- Provides header-only big-endian packing and unpacking helpers

---

# Architecture Overview

The system consists of:

- A fixed-size subscription table (no heap usage)
- A fixed-size scheduled TX table (no heap usage)
- A FreeRTOS RX task
- A FreeRTOS TX task
- Mutexes protecting subscription and schedule state
- Hardware mask filtering via TWAI
- Strict TX ID range enforcement



---

# Quick Start

## 1. Configure Hardware Filters

Must be called **before `init()`**.

```cpp
MSM_CAN::set_hardware_filters();               // TX-only (accept none)
MSM_CAN::set_hardware_filters(0x200);          // Accept one ID
MSM_CAN::set_hardware_filters(0x200, 0x2FF);   // Accept range (mask-block superset)
```

If no filter is configured, the default behaviour is TX-only.

---

## 2. Initialise

```cpp
MSM_CAN::init(RX_GPIO, TX_GPIO);
```

Initialisation performs:

- TWAI node creation
- Hardware filter application
- Node enable
- RX task creation
- TX task creation
- Subscription mutex creation
- Schedule mutex creation

Calling `init()` twice returns `ESP_ERR_INVALID_STATE`.

---

## 3. Subscribe to IDs

```cpp
MSM_CAN::subscribe(0x200, my_callback);
MSM_CAN::subscribe(0x201); // callback optional
```

Callback signature, if used:

```cpp
void my_callback(uint16_t id,
                 const uint8_t data[8],
                 uint32_t timestamp_ms);
```

Rules:

- Must be fast
- Must not block
- Must not store the pointer to `data`
- Always receives exactly 8 bytes

If the ID does not pass hardware filtering, `subscribe()` returns `ESP_ERR_INVALID_ARG`.

For polling-only users, each subscribed ID also caches its most recent received frame:

```cpp
uint8_t data[8];
uint32_t timestamp_ms = 0;

esp_err_t err = MSM_CAN::get(0x200, data, &timestamp_ms);
if (err == ESP_OK)
{
    // data / timestamp_ms contain the most recently received packet
}
```

`get()` returns:

- `ESP_OK` if a cached packet is available
- `ESP_ERR_NOT_FOUND` if the ID is not subscribed or no packet has been received yet
- `ESP_ERR_INVALID_ARG` if the arguments are invalid

---

## 4. Transmit

```cpp
uint8_t payload[8];
MSM_CAN::clear_payload(payload);

MSM_CAN::pack_u16(payload, 0, 1234);
MSM_CAN::send_msg(0x500, payload);
```

Transmission rules:

- ID must be within allowed TX ranges
- Payload must be exactly 8 bytes
- Encoding is big-endian
- `send_msg()` blocks until the TX task has completed the transmit request

Periodic transmit helpers:

```cpp
MSM_CAN::schedule(0x501, payload, 100);             // send every 100 ms
MSM_CAN::update_scheduled_payload(0x501, payload);  // update payload only
MSM_CAN::unschedule(0x501);                         // stop periodic transmit
```

---

# Transmission Policy

Allowed TX ID ranges (these can be modified in MSM_CAN.cpp, in future an API to edit them will be created):

```
0x100 - 0x1FF
0x500 - 0x5FF
```

This prevents accidental broadcasting into protected bus regions.

---

# Hardware Filtering

Mask filter logic:

```
(incoming_id & mask) == (filter_id & mask)
```

Range filtering uses a mask-block superset.

Important:

Hardware filtering may accept a superset of IDs.  
The software subscription table ensures only explicitly subscribed IDs trigger callbacks.

---

# Internal Behaviour

RX task behaviour:

- Blocks on `twai_node_receive`
- Rejects extended IDs
- Rejects non-8-byte frames
- Copies payload to local buffer
- Locks subscription table
- Updates cached packet/timestamp for the matching subscribed ID
- Copies callback pointer
- Unlocks
- Executes callback outside the lock if one was provided

No dynamic allocation occurs in the RX path.

---

# Thread Safety

The subscription and scheduled-TX tables are protected by FreeRTOS mutexes.

Protected operations:

- `subscribe()`
- `unsubscribe()`
- `get()`
- `schedule()`
- `update_scheduled_payload()`
- `unschedule()`
- RX callback lookup

The mutex is never held while executing user callbacks.

This prevents:

- Use-after-unsubscribe
- Partial entry reads
- Slot reuse race conditions

---

# Helper Functions

### Packing Helpers (Big Endian)

```cpp
pack_u8(data, index, value);
pack_i8(data, index, value);
pack_u16(data, index, value);
pack_i16(data, index, value);
pack_u32(data, index, value);
pack_float(data, index, value);
```

### Unpacking Helpers (Big Endian)

```cpp
unpack_u8(data, index);
unpack_i8(data, index);
unpack_u16(data, index);
unpack_i16(data, index);
unpack_u32(data, index);
unpack_float(data, index);
```

> **_NOTE:_** `float` decoding assumes IEEE-754 single precision

### Bit Manipulation

```cpp
set_bit(byte, bit_position, true_or_false);
check_flag(data, byte, bit_position);
```

### Payload Utility

```cpp
clear_payload(data);
```

---

# Error Handling

Common return values:

- `ESP_ERR_INVALID_STATE` → Called before init or invalid call order
- `ESP_ERR_INVALID_ARG` → Invalid ID or hardware filter mismatch
- `ESP_ERR_NO_MEM` → No subscription slots available
- `ESP_FAIL` → Internal failure

Application code should wrap calls with:

```cpp
ESP_ERROR_CHECK(...)
```

---

# Constraints

- Classic CAN only
- Standard 11-bit IDs only
- DLC must be 8
- Hardware filters must be configured before init
- RX callbacks must be fast and non-blocking


