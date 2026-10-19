# Smartcar vehicle control project

This repository contains the RT1064 vehicle firmware and the E35 desktop telemetry/control application.

- `firmware/demo/DEMO/mdk/rt1064.uvprojx`: Keil uVision project. Its required `user`, `code`, and vendor `libraries` sources are included with the original relative layout.
- `E35_machine/`: Python desktop application for E35 transparent UART telemetry, logging, IMU calibration, and vehicle controls. See its README and PROTOCOL documents.

The firmware project was built with Keil MDK 5.39 / ArmClang 6.21 for target `nor_sdram_zf_dtcm`. It built with 0 errors and 0 warnings on 2026-10-19. No MCU was connected for this build.
