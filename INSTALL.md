# **Installation Instructions**

This document provides the necessary steps to install and configure the development environment to work with STM32 boards (using PlatformIO with `libopencm3`) and LPC1769 (using MCU Expresso).

---

## **Prerequisites**

1. **Required Development Tools**:

   - [PlatformIO](https://platformio.org/install/ide) (integrated extension for Visual Studio Code).
   - [MCU Expresso IDE](https://www.nxp.com/mcuxpresso).
   - ARM GCC Compiler: Download from [ARM Developer](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain).
   - USB drivers for development boards:
     - STM32: ST-Link (download from [STMicroelectronics](https://www.st.com)).
     - LPC1769: NXP LinkServer or compatible drivers.
2. **Hardware**:

   - STM32 board (e.g., STM32F103C8T6).
   - LPC1769 board (NXP LPCXpresso1769).
3. **Cables and Adapters**:

   - Micro-USB cable for communication.
   - Compatible adapter or debugger (ST-Link for STM32, NXP LinkServer for LPC1769).

---

## **Installation Instructions**

### **STM32 - PlatformIO in Visual Studio Code**

1. **Install PlatformIO**:

   - Open Visual Studio Code.
   - Go to the Extensions section (`Ctrl+Shift+X`).
   - Search for and install `PlatformIO IDE`.
2. **Clone the repository**:

   - Open the terminal and run:
     ```bash
     git clone https://github.com/ICOMP-UNC/4DigiTech---Team-20.git
     cd 4DigiTech---Team-20
     cd STM32
     ```
3. **Configure the environment**:

   - Open the `platformio.ini` file and verify it is set up for your board:
     ```ini
     [env:genericSTM32F103C8]
     platform = ststm32
     board = genericSTM32F103C8
     framework = libopencm3
     upload_protocol = stlink
     debug_tool = stlink
     build_flags = -Og -g3
     ```
4. **Compile and upload the project**:

   - Compile the code with:
     ```bash
     platformio run
     ```
   - Upload the firmware to the board:
     ```bash
     platformio run --target upload
     ```
5. **Debugging** (optional):

   - Connect the ST-Link debugger and run:
     ```bash
     platformio debug
     ```

---

### **LPC1769 - MCU Expresso**

1. **Install MCU Expresso IDE**:

   - Download and install it from [NXP](https://www.nxp.com/mcuxpresso).
   - Configure the environment to use the ARM GCC compiler.
2. **Clone the repository**:

   - Open the terminal and run:
     ```bash
     git clone https://github.com/ICOMP-UNC/4DigiTech---Team-20.git
     cd 4DigiTech---Team-20
     cd STM32
     ```
3. **Import the project**:

   - Open MCU Expresso IDE.
   - Go to `File > Import > Existing Projects into Workspace`.
   - Select the folder of the cloned project.
4. **Compile the code**:

   - Right-click on the project in the project explorer.
   - Select `Build Project`.
5. **Upload the firmware**:

   - Connect the LPC1769 board to the PC.
   - Click the `Debug` button in the IDE.
   - Ensure the debugger (LinkServer) is correctly configured.
6. **Debugging**:

   - Set breakpoints in the code and use the IDE’s debugging tools to inspect the program flow.

---

## **Common Issues and Solutions**

1. **Device connection error**:

   - Ensure the USB drivers are installed correctly.
   - Verify the board is in programming mode (Bootloader or Debug).
2. **Compilation error**:

   - Confirm all dependencies are installed.
   - Check the library directories in `lib/` (for STM32) or the SDK configurations (for LPC1769).
3. **Board not detected**:

   - Replace the USB cable or use a different USB port (make sure to use a data cable, not a charge-only cable).
   - Ensure the board is powered on and properly connected.
   - For LPC1769:
     - First, disconnect the pins joined at JP3 and reconnect.

     ![1731853422937](https://drive.google.com/file/d/1B91R4Fd4a2y3AZ4Lca11ZHd0JePqkDGg/preview)

4. **Board detected by the PC but not by the development environment**:

   - **LPC1769**: Follow the configurations in the following link for Windows-specific setup: [Windows Configuration](https://support.microsoft.com/es-es/windows/no-se-puede-cargar-un-controlador-en-este-dispositivo-8eea34e5-ff4b-16ec-870d-61a4a43b3dd5).
   - **STM32**: Force a board reset by loading a project while holding the RESET button and releasing it during the upload. Repeat if necessary.

---

## **Additional Resources**

- [PlatformIO Documentation](https://docs.platformio.org/en/latest/)
- [MCU Expresso IDE Documentation](https://www.nxp.com/design/software/development-software/mcuxpresso-ide:MCUXpresso-IDE)
- [LibOpenCM3 API Reference](https://libopencm3.org/)

---
