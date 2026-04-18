# System Overview

A real-time, edge-based acoustic monitoring system designed for high-performance sound frequency detection and remote alerting. This project leverages the STM32 architecture to perform on-device digital signal processing (DSP) and transmits anomaly data over Wi-Fi.

## System Architecture

The project is structured to minimize CPU overhead by utilizing hardware peripherals for data movement and transformation.

- **Acquisition:** A MEMS Microphone captures ambient sound, transmitting a PDM (Pulse Density Modulation) stream to the MCU.

- **Ingestion:** The DFSDM (Digital Filter for Sigma-Delta Modulators) peripheral interprets the 1s and 0s, converting them into PCM audio samples.

- **Data Movement:** DMA (Direct Memory Access) transfers these samples into RAM buffers without CPU intervention.

- **Buffering Strategy:** Uses a circular buffer with Half-Transfer and Full-Transfer interrupts. This allows the CPU to process one half of the buffer while the DMA continues filling the other, ensuring zero data loss.

- **Processing:** The CPU executes a Fast Fourier Transform (FFT) to analyze the frequency spectrum of the captured audio.

- **Decision Logic:** An algorithm monitors frequency magnitudes over a defined time threshold. If a specific frequency signature is detected, an alert is triggered.

- **Communication:** Alerts are dispatched via an external Wi-Fi module, which routes data through a local gateway to a remote server for logging or notification.
