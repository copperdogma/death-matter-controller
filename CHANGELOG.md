# Changelog

## [2025-10-29] - UART handshake reliability and pin remap

### Added
- Boot and fabric handshake frames with retry timers to keep the ESP32-C3 and WROVER in sync.

### Changed
- Moved the inter-board UART to adjacent GPIO5/GPIO6 pins on the ESP32-C3 SuperMini for cleaner wiring.
- Updated status LED feedback to flash twice on every UART transmission.

### Fixed
- README now documents the new UART handshake protocol and wiring guidance.
