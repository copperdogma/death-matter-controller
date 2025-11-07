# Changelog

## [2025-10-29] - UART handshake reliability and pin mapping

### Added
- Boot and fabric handshake frames with retry timers to keep the ESP32-C3 and WROVER in sync.

### Changed
- Reaffirmed inter-board UART on GPIO21 (TX) / GPIO20 (RX) on the ESP32-C3 SuperMini.
- Updated status LED feedback to flash twice on every UART transmission.

### Fixed
- README now documents the new UART handshake protocol and wiring guidance.
