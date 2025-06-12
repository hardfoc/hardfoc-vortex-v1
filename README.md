# Create the README.md content
readme_content = """
# 🚀 Welcome to **HardFOC** – Hardware-Accelerated Field Oriented Control

**HardFOC** is a high-performance, hardware-centric Field Oriented Control (FOC) framework designed for embedded systems. Unlike software-only solutions, HardFOC leverages hardware peripherals (PWM, ADC, timers, DMA, etc.) to achieve ultra-efficient, real-time motor control with minimal CPU overhead.

> 💡 **HardFOC** stands for **Hardware-based FOC**, not because it's hard — but because it's fast, portable, and built for real-world embedded applications.

---

## 🎯 Project Goals

- ✅ **Hardware-accelerated FOC** using hardware-based FOC drivers
- ✅ **Portable HAL architecture** for rapid adaptation to new MCUs and Boards
- ✅ **Modular design** supporting BLDC, PMSM, and stepper motors
- ✅ **Sensor-agnostic**: supports encoders, Hall sensors, sensorless control
- ✅ **Driver-flexible**: works with gate drivers, integrated drivers, and smart FETs
- ✅ **Open-source and community-driven**

---

## 🔧 Why HardFOC?

| Feature | HardFOC | SimpleFOC |
|--------|---------|-----------|
| Hardware abstraction layer (HAL) | ✅ Portable & modular | ⚠️ Arduino-centric |
| Real-time performance | ✅ Hardware-timed | ⚠️ Software-loop limited |
| MCU support | ✅ STM32, RP2040, ESP32, etc. | ✅ Arduino-compatible |
| Sensor support | ✅ Modular & extensible | ✅ Good |
| Community | 🚧 Growing | 🌍 Established |

---

## 📦 Key Components

- **🧠 Core FOC Engine** – Fast, deterministic control loop using hardware implementation
- **🔌 HAL Layer** – Abstracts MCU & Boards-specific peripherals for easy portability
- **📍 Position & Velocity Sensors** – Encoder, Hall, sensorless (back-EMF, observer-based)
- **📊 Debug & Tuning Tools** – Serial interface, real-time telemetry, waveform capture

---

## 🧪 TMC9660 Microcontroller

The **TMC9660** microcontroller is at the heart of the HardFOC framework, providing extensive hardware acceleration features that make it ideal for real-time motor control applications. Key features include:

- **Integrated FOC Engine**: Hardware-accelerated Field Oriented Control for ultra-fast response times.
- **PWM Generation**: High-resolution PWM outputs for precise motor control.
- **ADC Integration**: Synchronized ADC sampling for accurate current and voltage measurements.
- **DMA Support**: Direct Memory Access for efficient data transfer with minimal CPU intervention.
- **Advanced Timers**: Multiple hardware timers for precise control loop timing and event scheduling.
- **Communication Interfaces**: Support for UART, SPI, I2C, and CAN for robust communication with sensors and other peripherals.

The TMC9660's hardware acceleration capabilities enable the HardFOC framework to achieve unparalleled performance and efficiency, making it suitable for a wide range of applications, from robotics to industrial automation.

---

## 📁 Repository Structure

```
hardfoc-*/              # Top-Level (board-specific)
├── hf-hal-*/           # Hardware abstraction layer (board-specific)\
├── hf-middleware-*/    # Middleware layer (board-specific)\
├── src/                # Main (board-specific)\
├── docs/               # Documentation and guides
└── tools/              # CLI tools, tuning utilities
```

---

## 📚 Documentation

- **Getting Started**: docs.hardfoc.dev/getting-started
- **API Reference**: docs.hardfoc.dev/api
- **Porting Guide**: docs.hardfoc.dev/porting
- **Hardware Setup**: docs.hardfoc.dev/hardware

---

## 🌐 Community & Support

- 💬 **Forum**: community.hardfoc.dev
- 🛠️ **GitHub Discussions**: github.com/hardfoc/discussions
- 📺 **YouTube Tutorials**: youtube.com/@hardfoc
- 🧠 **Knowledge Base**: kb.hardfoc.dev

---

## 🧪 Example Use Cases

- High-speed BLDC control for drones and robotics
- Precision stepper motor control with FOC
- Sensorless motor control for compact designs
- Industrial automation and CNC applications

---

## 🤝 Contributing

We welcome contributions from developers, engineers, and enthusiasts! Whether you're porting to a new MCU, improving documentation, or building new drivers — your help is appreciated.

Start here 👉 CONTRIBUTING.md

---

## 📦 Installation

```bash
git clone https://github.com/hardfoc/HardFOC.git
cd HardFOC
# Follow platform-specific setup instructions in /docs
```

---

## 📣 Stay Connected

!GitHub Stars  
!YouTube  
!Discord

---


