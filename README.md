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

This library:

- Transmits and receives raw `uint8_t[8]` payloads
- Supports periodic scheduled transmission
- Enforces TX ID ranges
- Manages hardware filtering
- Provides safe callback-based reception
- Caches the latest received packet per subscribed ID
- Protects internal state with FreeRTOS mutexes
- Uses fixed-size subscription and scheduled-TX tables
- Provides header-only big-endian packing and unpacking helpers

---

# Architecture Overview

The system consists of:

- A configurable fixed-size subscription table
- A fixed-size scheduled TX table (no heap usage)
- A FreeRTOS RX task
- A FreeRTOS TX task
- Mutexes protecting subscription and schedule state
- Hardware mask filtering via TWAI
- Strict TX ID range enforcement

The subscription table defaults to 64 entries. Projects can override this at
compile time before including the header, or by adding a compiler definition:

```cpp
#define MSM_CAN_MAX_SUBS 128
#include "MSM_CAN.hpp"
```

For ESP-IDF/CMake projects, this can also be supplied as a compile definition:

```cmake
target_compile_definitions(${COMPONENT_LIB} PUBLIC MSM_CAN_MAX_SUBS=128)
```

Increasing `MSM_CAN_MAX_SUBS` raises static RAM usage because each subscription
slot stores callback metadata and one cached 8-byte receive packet.

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
MSM_CAN::LatestPacket packet = MSM_CAN::get(0x200);
if (packet.has_packet)
{
    uint16_t value = MSM_CAN::unpack_u16(packet.data, 0);
    uint32_t timestamp_ms = packet.timestamp_ms;
}
```

`get()` returns a zeroed `LatestPacket` with `has_packet == false` when the
driver is not initialised, the ID is invalid, the ID is not subscribed, or no
packet has been received yet.

The packet data is copied out of the internal cache before `get()` returns.

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

- `ESP_ERR_INVALID_STATE` -> Called before init or invalid call order
- `ESP_ERR_INVALID_ARG` -> Invalid ID or hardware filter mismatch
- `ESP_ERR_NO_MEM` -> No subscription slots available
- `ESP_FAIL` -> Internal failure

Application code should wrap `esp_err_t`-returning calls with:

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


