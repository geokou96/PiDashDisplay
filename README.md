DashDisplay: Software-Defined Automotive Instrument Cluster & Open Telemetry Architecture

DashDisplay is a low-cost, software-defined automotive instrument cluster and real-time telemetry platform built on a Raspberry Pi 4. Designed to bridge standalone ECUs (e.g., Link G4+) with heterogeneous OEM chassis electronics (e.g., Mazda RX-8 with swapped Nissan SR20DET powertrain), 
it provides real-time driver visualization, digital signal isolation, active CAN-bus sensor expansion, and an onboard time-series database (TSDB) paired with an embedded Grafana observability stack.

This repository contains the complete source code, JSON signal mapping profiles, database initialization scripts, systemd service configurations, and hardware schematic documentation corresponding to the paper:

Integrated CAN-Bus System: A Software-Defined Multi-Threaded Instrument Cluster and Open Observability Telemetry Architecture for Vehicle Performance Optimization (MDPI Electronics, 2026).

Key Features:

        Deterministic Multi-Threaded Engine: Python 3.11 runtime isolating high-rate SocketCAN ingestion, MCP3008 ADC sampling, NEO-6M GNSS parsing, UI rendering, and database logging.
        Kernel-Level CAN Interface: Uses Linux SocketCAN (mcp251x kernel driver) at 500 kbps to guarante no frame loss.
        Adaptive UI Queue Throttling: Non-blocking Tkinter consumer loop throttled to max 25 queued updates per 20ms cycle, guaranteeing a stable 50 FPS rendering rate under heavy bus loads.
        Active CAN Expansion Gateway: Samples MAX31855 K-type thermocouple over Software SPI (10Hz) and broadcasts latched 8-byte frames (ID 0x457) at 20 Hz to standalone ECUs lacking thermocouple interfaces.
        JSON-Driven Signal Parser: Decodes telemetry streams, endianness, linear scaling, and bitwise diagnostic masks (e.g., MIL, Low Oil Pressure, SRS) dynamically via external JSON profiles without code recompilation.
        Onboard Observability Stack: Local MariaDB relational TSDB storing sensor logs in daily partitioned tables (sensor_log_YYYYMMDD) with batched SQL commits (200rows/ 50ms), queried live by Grafana via local HTTP.
        Automated Kinematics & Lap Timer: Haversine formula GPS lap timer (12m radius target) and real-time speed cross-validation reporting V_{error} RMSE (1.84 km/h).
        Hardware Signal Isolation & Safe Shutdown: Optocoupler level-shifting (OP71A04), diode multiplexing (1N4007), and ACC-triggered delayed shutdown relay protecting file system integrity.
