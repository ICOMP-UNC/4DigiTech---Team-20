# 4DigiTech---Team-20  
# **Air Quality Monitoring and Fan Control System**  

This project implements an air quality monitoring and fan control system using two microcontrollers: LPC1769 and STM32. The system enables real-time simulation and management of air quality conditions with advanced features such as airflow monitoring, Air Quality Index (AQI) calculation, and dynamic fan speed control.  

## **Overview**  

The system is designed to integrate hardware and software seamlessly, leveraging:  
- **LPC1769** for fan control, AQI calculation, and filter condition visualization using LEDs.  
- **STM32** for simulating the PSM5003 sensor and generating realistic air quality data.  

This modular approach ensures scalability and simplifies future integrations, such as real sensors or enhanced communication protocols.  

---  

## **Key Features**  

### **LPC1769**  
- **Airflow Monitoring:**  
  - Detects blockages in the air filter by analyzing airflow rate.  
  - Adjusts the brightness of a DAC-connected LED to indicate the filter's condition.  

- **Fan Control (PWM):**  
  - Dynamically adjusts fan speed based on the AQI.  
  - Enhances air purification efficiency.  

- **UART Communication:**  
  - Receives simulated data from the STM32.  
  - Implements checksum verification to ensure data integrity.  

- **AQI Calculation:**  
  - Converts PM2.5 concentrations into AQI values to control the fan.  

---  

### **STM32**  
- **PSM5003 Sensor Simulation:**  
  - Generates a 32-byte data frame with adjustable PM2.5 values.  

- **PM2.5 Control with Potentiometer:**  
  - Manually adjusts simulated concentrations (0-300 µg/m³).  

- **FreeRTOS:**  
  - Leverages tasks to simulate sensor functionality and ensure efficient operations.  
  - Implements *High-Water Mark* for stack usage monitoring.  

---  

## **Project Structure**  

```plaintext
.
├── src/
│   ├── LPC1769/         # Code for LPC1769 microcontroller
│   ├── STM32/           # Code for STM32 microcontroller
├── docs/
│   ├── INSTALL.md       # Installation guide
│   ├── Doxygen/         # Automatically generated documentation
├── tests/
│   ├── manual/          # Manual tests performed
└── README.md            # This file
```
## **Installation** ##
For detailed installation instructions, refer to the [INSTALL.md](INSTALL.md) file in the docs/ directory.