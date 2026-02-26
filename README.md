# MSM_CAN - Manchester Stinger Motorsports CAN Library

Classic CAN 2.0 library for ESP32 (ESP-IDF, TWAI on-chip driver).

Designed for Manchester Stinger Motorsports by James Platt 

---

## Bus Configuration

- Bitrate: **1 Mbit/s**
- Protocol: **Classic CAN 2.0**
- Identifier: **11-bit standard IDs only**
- Payload length: **Always 8 bytes (DLC = 8)**

This library is intentionally strict. Much of the Bus Configuration is defined by the Haltech Elite 1500 ECU.
---

# Design Philosophy

This library:

- Transmits and receives raw `uint8_t[8]` payloads
- Enforces TX ID ranges
- Manages hardware filtering
- Provides safe callback-based reception
- Protects internal state with a mutex
- Avoids dynamic memory allocation

For now signal packing/unpacking is the responsibility of the application.

---

# Architecture Overview

The system consists of:

- A fixed-size subscription table (no heap usage)
- A FreeRTOS RX task
- A mutex protecting subscription state
- Hardware mask filtering via TWAI
- Strict TX ID range enforcement

All reception is callback-driven.

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
MSM_CAN::init(GPIO_NUM_5, GPIO_NUM_4);
```

Initialisation performs:

- TWAI node creation
- Hardware filter application
- Node enable
- RX task creation
- Subscription mutex creation

Calling `init()` twice returns `ESP_ERR_INVALID_STATE`.

---

## 3. Subscribe to IDs

```cpp
MSM_CAN::subscribe(0x200, my_callback);
```

Callback signature:

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
- Transmission blocks until complete

---

# Transmission Policy

Allowed TX ID ranges:

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
- Copies callback pointer
- Unlocks
- Executes callback outside the lock

No dynamic allocation occurs in the RX path.

---

# Thread Safety

The subscription table is protected by a FreeRTOS mutex.

Protected operations:

- `subscribe()`
- `unsubscribe()`
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
pack_u16(data, index, value);
pack_u32(data, index, value);
```

### Bit Manipulation

```cpp
set_bit(byte, bit_position, true_or_false);
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

---

# Planned Future Additions

- get() function that returns most recent received packet
- schedule() that schedules a packet to be sent every x seconds
- unpack() helper functions 


