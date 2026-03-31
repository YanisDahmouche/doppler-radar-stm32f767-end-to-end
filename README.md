# DopplerShield-F767: End-to-End 24.125 GHz Radar Signal Chain

An end-to-end Doppler radar project built around an **STM32F767ZI**, covering the full path from the **analog front-end** to **embedded acquisition/processing**, **OLED local feedback**, and two PC tools:

- `pc_scope`: real-time time-domain + FFT scope UI
- `pc_analyzer`: advanced UDP-based DSP analyzer (FFT/CFAR/Kalman/tracking)

This repository is intended as a complete technical presentation of the project architecture and implementation.

## Project scope

The system measures Doppler shifts from a 24.125 GHz radar front-end and estimates target speed in real time.

It includes:

- Analog IF chain ingestion by ADC
- Embedded DMA-based continuous sampling on STM32
- Embedded FFT path + legacy UART output
- Ethernet UDP raw hop streaming to PC
- OLED status/telemetry display
- Desktop real-time visualization (`pc_scope`)
- Desktop advanced detection/tracking pipeline (`pc_analyzer`)

## System architecture (full chain)

`Radar analog front-end (IF/baseband)`  
`-> STM32 ADC1 @ 100 kHz (DMA circular)`  
`-> half/full DMA callbacks (1024-sample hops)`  
`-> embedded buffering + optional embedded FFT metrics`  
`-> UDP fragmentation + Ethernet transmission (port 5555)`  
`-> PC reassembly + sliding analysis window`  
`-> Hann + FFT + CFAR + robustification + Kalman`  
`-> Live operator UI (scope / FFT / spectrogram / tracks)`  
`+ Local OLED feedback on STM32`

## Hardware and embedded firmware

Main firmware file: `Core\Src\main.c`

Key embedded blocks:

- **ADC1 + DMA2 Stream0** for uninterrupted acquisition
- **TIM2 trigger** driving ADC sampling at **100 kHz**
- **FFT pipeline (CMSIS DSP)** with DC removal and windowing
- **Doppler-to-speed conversion** tuned for 24.125 GHz radar
- **Ethernet packetization** with raw UDP frame handling
- **UART3 legacy output**
- **SSD1306 OLED over I2C1** for local status rendering
- **DAC + TIM4 path** kept in project for signal/reference generation/testing

Representative firmware constants (from `main.c`):

- `ADC_BUF_SIZE = 2048`
- `FFT_SIZE = 4096`
- `FS = 100000.0f`
- `MIN_BIN = 2`, `MAX_BIN = 250`
- UDP destination/source port `5555`

## Analog front-end role

The analog front-end provides the IF/baseband Doppler signal entering ADC1 (PA0 path).  
The digital chain assumes this analog signal is preconditioned (gain/filtering/biasing) for safe and informative ADC capture.

In practical terms:

- Front-end quality directly impacts noise floor and dynamic range
- LPF behavior bounds usable Doppler span
- Offset and hum components are handled in DSP (DC removal, mains filtering/notch options in PC analyzer)

## OLED role (embedded UX)

The SSD1306 OLED acts as local instrumentation:

- system/radar state visibility without a PC
- periodic updates (low overhead)
- quick verification of acquisition/processing health

This is useful for bring-up, field checks, and standalone diagnostics.

## PC applications

### 1) `pc_scope` (serial scope)

Primary role: quick visualization and debugging of streamed radar behavior.

- Real-time waveform scope
- FFT panel and spectrogram panel
- Trigger-style viewing utilities
- Lightweight immediate UI for bench validation

Entry file: `pc_scope\radar_scope.cpp`  
Build system: `pc_scope\CMakeLists.txt` and `pc_scope\build.bat`

### 2) `pc_analyzer` (advanced analyzer)

Primary role: robust real-time detection/estimation from UDP hops.

- UDP hop reassembly (fragmented ADC packets)
- Sliding time window reconstruction
- Configurable FFT (large sizes for fine bin spacing)
- CA-CFAR thresholding
- Noise-floor adaptation and notch/hum mitigation controls
- Peak extraction and confidence metrics
- Kalman filtered kinematic estimation
- Multiple display layouts (FFT/scope/spectrogram focused views)

Entry file: `pc_analyzer\radar_analyzer.cpp`  
Build system: `pc_analyzer\CMakeLists.txt` and `pc_analyzer\build.bat`

## Repository structure

```text
Core\Src\main.c                 # STM32 firmware core logic
Core\Src\ssd1306.c              # OLED driver implementation
Core\Inc\ssd1306.h              # OLED driver interface
Drivers\                        # STM32 HAL/CMSIS stack
Middlewares\                    # Middleware components
pc_scope\                       # PC scope application (ImGui/OpenGL/GLFW)
pc_analyzer\                    # PC analyzer application (UDP + DSP + UI)
Documentation_main_c.tex        # Full firmware technical documentation
Documentation_pc_analyzer_fft.tex # Deep DSP/FFT theory + implementation doc
projet_L3.ioc                   # CubeMX project config
```

## Build and run

## STM32 firmware

Use **STM32CubeIDE** with the project root (`projet_L3.ioc`).

Typical flow:

1. Open project in CubeIDE
2. Build firmware
3. Flash STM32F767ZI
4. Verify Ethernet/UART/OLED behavior

## PC tools (Windows)

Both apps use CMake and fetch dependencies (GLFW, Dear ImGui).

For `pc_scope`:

```bat
cd pc_scope
build.bat
```

For `pc_analyzer`:

```bat
cd pc_analyzer
build.bat
```

## Network setup for analyzer

Current project defaults are static:

- STM32: `192.168.1.10`
- PC: `192.168.1.100` (same subnet)
- UDP port: `5555`

Ensure firewall rules allow inbound UDP on the chosen port.

## Documentation

- `Documentation_main_c.tex` / PDF: embedded architecture and `main.c` deep dive
- `Documentation_pc_analyzer_fft.tex` / PDF: DSP chain from acquisition to final speed estimate, with theory and formulas



### Topics (suggestion)

`stm32` `stm32f7` `stm32f767zi` `radar` `doppler-radar` `signal-processing` `fft` `cfar` `kalman-filter` `embedded-systems` `adc` `dma` `oled` `ethernet` `udp` `immediate-mode-gui` `imgui` `glfw` `opengl` `cplusplus`
