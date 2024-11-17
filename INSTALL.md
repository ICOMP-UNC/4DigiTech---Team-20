# **Instrucciones de Instalación**

Este documento proporciona los pasos necesarios para instalar y configurar el entorno de desarrollo para trabajar con las placas STM32 (usando PlatformIO con `libopencm3`) y LPC1769 (usando MCU Expresso).

---

## **Requisitos previos**

1. **Herramientas de desarrollo necesarias**:

   - [PlatformIO](https://platformio.org/install/ide) (extensión integrada en Visual Studio Code).
   - [MCU Expresso IDE](https://www.nxp.com/mcuxpresso).
   - Compilador ARM GCC: Descarga desde [ARM Developer](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain).
   - Controladores USB para placas de desarrollo:
     - STM32: ST-Link (descarga de [STMicroelectronics](https://www.st.com)).
     - LPC1769: NXP LinkServer o drivers compatibles.
2. **Hardware**:

   - Placa STM32 (ejemplo: STM32F103C8T6).
   - Placa LPC1769 (NXP LPCXpresso1769).
3. **Cables y adaptadores**:

   - Cable micro-USB para comunicación.
   - Adaptador o depurador compatible (ST-Link para STM32, NXP LinkServer para LPC1769).

---

## **Instrucciones de instalación**

### **STM32 - PlatformIO en Visual Studio Code**

1. **Instalar PlatformIO**:

   - Abre Visual Studio Code.
   - Ve a la sección de extensiones (`Ctrl+Shift+X`).
   - Busca e instala `PlatformIO IDE`.
2. **Clonar el repositorio**:

   - Abre la terminal y ejecuta:
     ```bash
     git clone https://github.com/usuario/proyecto-stm32.git
     cd proyecto-stm32
     ```
3. **Configurar el entorno**:

   - Abre el archivo `platformio.ini` y verifica que esté configurado para tu placa:
     ```ini
     [env:genericSTM32F103C8]
     platform = ststm32
     board = genericSTM32F103C8
     framework = libopencm3
     upload_protocol = stlink
     debug_tool = stlink
     build_flags = -Og -g3
     ```
4. **Compilar y cargar el proyecto**:

   - Compila el código con:
     ```bash
     platformio run
     ```
   - Carga el firmware en la placa:
     ```bash
     platformio run --target upload
     ```
5. **Depuración** (opcional):

   - Conecta el depurador ST-Link y ejecuta:
     ```bash
     platformio debug
     ```

---

### **LPC1769 - MCU Expresso**

1. **Instalar MCU Expresso IDE**:

   - Descarga e instala desde [NXP](https://www.nxp.com/mcuxpresso).
   - Configura el entorno para usar el compilador ARM GCC.
2. **Clonar el repositorio**:

   - Abre la terminal y ejecuta:
     ```bash
     git clone https://github.com/usuario/proyecto-lpc1769.git
     cd proyecto-lpc1769
     ```
3. **Importar el proyecto**:

   - Abre MCU Expresso IDE.
   - Ve a `File > Import > Existing Projects into Workspace`.
   - Selecciona la carpeta del proyecto clonado.
4. **Compilar el código**:

   - Haz clic derecho sobre el proyecto en el explorador de proyectos.
   - Selecciona `Build Project`.
5. **Cargar el firmware**:

   - Conecta la placa LPC1769 al PC.
   - Haz clic en el botón `Debug` en el IDE.
   - Verifica que el depurador (LinkServer) esté correctamente configurado.
6. **Depuración**:

   - Establece breakpoints en el código y usa las herramientas de depuración del IDE para inspeccionar el flujo del programa.

---

## **Problemas comunes y soluciones**

1. **Error de conexión al dispositivo**:

   - Verifica que los controladores USB estén instalados correctamente.
   - Asegúrate de que la placa esté en modo de programación (Bootloader o Debug).
2. **Error de compilación**:

   - Confirma que todas las dependencias estén instaladas.
   - Revisa los directorios de las bibliotecas en `lib/` (para STM32) o las configuraciones del SDK (para LPC1769).
3. **Placa no detectada**:

   - Cambia el cable USB o usa un puerto USB diferente. (Recordar usar cable de datos verificar que no sea solo de carga)
   - Asegúrate de que la placa esté encendida y conectada correctamente.
   - LPC1769:

     - 1ro probar desconectando los pines unido en JP3 y volver a conectar

    ![1731853422937](image/install/1731853422937.png) 

    4.**Caso placa detectada por el equipo pero no por el entrono de desarrollo**

* LPC 1769: Seguir las configuraciones del siguiente link solo caso [configuración widows](https://support.microsoft.com/es-es/windows/no-se-puede-cargar-un-controlador-en-este-dispositivo-8eea34e5-ff4b-16ec-870d-61a4a43b3dd5).
* STM32: Forzar un reseteo de la placa cargando un proyecto mientras se mantiene apretado el boton de RESET y soltarlo cuando este por cargar el programa. (Probar repetidamente)

---

## **Recursos adicionales**

- [Documentación de PlatformIO](https://docs.platformio.org/en/latest/)
- [Documentación de MCU Expresso IDE](https://www.nxp.com/design/software/development-software/mcuxpresso-ide:MCUXpresso-IDE)
- [LibOpenCM3 API Reference](https://libopencm3.org/)

---
