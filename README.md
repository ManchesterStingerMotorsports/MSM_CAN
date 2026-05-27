# MSM_CAN Oxidation

Rust migration branch for MSM_CAN.

This branch is a minimal `esp-hal` starting point for replacing the old C++
ESP-IDF implementation. It does not contain the CAN library yet.

## Current Status

- Target stack: Rust `no_std` with `esp-hal`
- Target chip: ESP32
- Firmware target: `xtensa-esp32-none-elf`
- CAN/TWAI driver: not implemented yet

## First Build

Install the ESP Rust tooling, then check the project:

```powershell
cargo check
```

To flash later, install `espflash` and use:

```powershell
cargo espflash flash --monitor
```

## Notes

The old C++ sources have intentionally been removed on this branch. Use `main`
for the previous ESP-IDF/C++ implementation.
