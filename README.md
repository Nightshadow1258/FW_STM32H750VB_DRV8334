# Motor Controller Board - Firmware
Firmware for the custom Motor Controller Board built on the Zephyr RTOS. This repository contains the configuration, driver integrations, and application code required to drive a BLDC motor and perform synchronous phase current sampling using low-level STM32 HAL calls combined with Zephyr's devicetree infrastructure.

## Hardware Overview
- MCU: STM32H750VB (ARM Cortex-M7 @ up to 480 MHz)
- Clock Source: 16 MHz external HSE crystal
- Gate Driver Unit (GDU): 3-phase gate driver with current sense amplifiers (CSA)
- Current Sensing: 3x inline shunt resistors with 16-bit ADC sampling
- Peripherals Used:
	- TIM1: Advanced control timer (center-aligned PWM generation, dead-time insertion, ADC triggering)
	- ADC1 / ADC2 / ADC3: Synchronized 16-bit sampling triggered on TIM1 overflow
	- UART Shell: Debugging and interactive measurements

## Current Features & Implementation Status
- Clock Tree Setup: Configured via Zephyr devicetree (.dts) running at 480 MHz CPU clock / 120 MHz timer clock.
- Complementary 3-Phase PWM:
	- Configured on TIM1 (CH1/CH1N, CH2/CH2N, CH3/CH3N)
	- 20 kHz center-aligned mode (up/down counting)
	- Direct HAL hardware register configuration for precise timing and preload behavior
	- Configurable hardware dead-time insertion (set to 500 ns)
- Synchronized Phase Current Sampling:
	- Simultaneous 3-channel single-ended 16-bit ADC conversions triggered by TIM1 overflow events
	- Direct memory transfer via DMA (circular mode).
	- Automated offset calibration routine at startup (32-sample averaging per phase)
- Developer Tools:
	- Interactive Zephyr UART Shell interface for live testing and configuration
	- Statistical noise analysis shell tool (captures raw CSV output for standard deviation and mean calculations)

## Getting Started
### Prerequisites
- [Zephyr SDK](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) (v3.x or higher)
- west meta-tool
- ARM GNU Toolchain (`arm-none-eabi-gcc`)
- ST-Link or J-Link Probe for programming

### Building & Flashing
```
# Initialize and update west workspace (if not already done)
west init -l .
west update

# Build firmware for the motor controller target
west build -b your_board_target_name

# Flash the target MCU
west flash
```

## Roadmap / Future Work
- [ ] BLDC Motor Spin: Implement commutation logic and Field-Oriented Control (FOC) or 6-step commutation
- [ ] Position Sensor Integration: Read and process mechanical position data from the Motor Position Sensor (MPS)
- [ ] Closed-Loop Control: Implement PI current loops and speed/position controllers
- [ ] Thermal Monitoring: Enable NTC temperature sensor readings for thermal protection
- [ ] Communication Protocols:
	- [ ] Implement and test CAN bus messaging
	- [ ] Enable Ethernet interface
- [ ] Storage & Bootloader:
	- [ ] Integrate MCUboot for Execute-In-Place (XIP) execution on external flash
	- [ ] Store calibration and board-specific configuration data in external flash memory

# Related Resources

- Hardware Repository: [Motor Controller Board Hardware](https://github.com/Nightshadow1258/HW_Motor_Controller)
- Blog Posts:
	- [Part I - Hardware Design & Bringup](https://blog.preisch.cloud/diy-motor-controller-part-i-hardware-design-bringup/)
	- [Part II - Timer & ADC Configuration](https://blog.preisch.cloud/diy-motor-controller-board-part-ii-timer-adc-configuration/)

# License

This project is licensed under the MIT License.
