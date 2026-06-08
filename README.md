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

- Transmits `TxFrame` values and receives timestamped `RxFrame` values
- Supports periodic scheduled transmission
- Enforces TX ID ranges
- Manages hardware filtering
- Provides safe callback-based reception
- Caches the latest received frame per subscribed ID
- Protects internal state with FreeRTOS mutexes
- Uses fixed-size subscription and scheduled-TX tables
- Provides header-only big-endian packing and unpacking helpers

---

# Architecture Overview

The system consists of:

- A configurable fixed-size subscription table
- A configurable fixed-size scheduled TX table (no heap usage)
- A FreeRTOS RX task
- A FreeRTOS TX task
- Mutexes protecting subscription and schedule state
- Hardware mask filtering via TWAI
- Strict TX ID range enforcement

The subscription table defaults to 64 entries and the scheduled TX table
defaults to 32 entries. Projects can override these at compile time before
including the header, or by adding compiler definitions:

```cpp
#define MSM_CAN_MAX_SUBS 128
#define MSM_CAN_MAX_SCHEDULED_TX 64
#include "MSM_CAN.hpp"
```

For ESP-IDF/CMake projects, this can also be supplied as a compile definition:

```cmake
target_compile_definitions(${COMPONENT_LIB} PUBLIC
    MSM_CAN_MAX_SUBS=128
    MSM_CAN_MAX_SCHEDULED_TX=64
)
```

Increasing `MSM_CAN_MAX_SUBS` raises static RAM usage because each subscription
slot stores callback metadata and one cached 8-byte receive frame. Increasing
`MSM_CAN_MAX_SCHEDULED_TX` raises static RAM usage because each scheduled TX
slot stores the ID, payload, period, and next due time.

---

# Frame Types

MSM_CAN uses two frame structs:

```cpp
struct TxFrame
{
    uint16_t id;
    uint8_t data[8];
};

struct RxFrame
{
    uint16_t id;
    uint8_t data[8];
    uint32_t timestamp_ms;
};
```

`TxFrame` is used for anything the application sends. It contains only the CAN
identifier and the 8-byte payload because TX frames do not have a receive
timestamp.

`RxFrame` is used for anything the driver receives. It contains the CAN
identifier, a copied 8-byte payload, and the driver timestamp in milliseconds.

Both frame types use standard 11-bit CAN IDs. The `data` field is always exactly
8 bytes, matching this library's fixed DLC policy. The packing and unpacking
helpers operate directly on the `data` field:

```cpp
MSM_CAN::TxFrame tx = {};
tx.id = 0x500;
MSM_CAN::pack_u16(tx.data, 0, 1234);

MSM_CAN::RxFrame rx = {};
uint16_t value = MSM_CAN::unpack_u16(rx.data, 0);
```

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
void my_callback(const MSM_CAN::RxFrame& frame);
```

The callback receives a const reference to a temporary `RxFrame` owned by the RX
task. Read it during the callback, or copy out any fields that must live longer.

Rules:

- Must be fast
- Must not block
- Must not store the reference to `frame`
- May copy `frame.data` if it needs to keep the payload
- Always receives exactly 8 bytes

If the ID does not pass hardware filtering, `subscribe()` returns `ESP_ERR_INVALID_ARG`.

For polling-only users, each subscribed ID also caches its most recent received frame:

```cpp
MSM_CAN::RxFrame frame = {};
if (MSM_CAN::get(0x200, frame) == ESP_OK)
{
    uint16_t value = MSM_CAN::unpack_u16(frame.data, 0);
    uint32_t timestamp_ms = frame.timestamp_ms;
}
```

`get()` writes the latest cached `RxFrame` and returns `ESP_OK` when a frame is
available. It returns `ESP_ERR_INVALID_STATE` when the driver is not initialised,
`ESP_ERR_INVALID_ARG` for an invalid ID, and `ESP_ERR_NOT_FOUND` when the ID is
not subscribed or no frame has been received yet.

The frame data is copied out of the internal cache before `get()` returns.

---

## 4. Transmit

```cpp
MSM_CAN::TxFrame frame = {};
frame.id = 0x500;

MSM_CAN::pack_u16(frame.data, 0, 1234);
MSM_CAN::send_msg(frame);
```

Transmission rules:

- ID must be within allowed TX ranges
- Payload must be exactly 8 bytes
- Encoding is big-endian
- `send_msg()` blocks until the TX task has completed the transmit request

Periodic transmit helpers:

```cpp
MSM_CAN::schedule(frame, 100);                    // send every 100 ms
MSM_CAN::update_scheduled_payload(frame);         // update payload only
MSM_CAN::unschedule(frame.id);                    // stop periodic transmit
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
- Updates cached frame/timestamp for the matching subscribed ID
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
- `ESP_ERR_NOT_FOUND` -> No cached RX frame is available for that ID
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


