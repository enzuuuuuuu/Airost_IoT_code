The code is written using HAL libraries built in STM32 CubeIDE, no environment need to be install prior to uploading the code.

**UPLOADING USING UART**
1) Navigate to the Debug folder inside your project directory.
2) Locate and copy the generated .bin file.
3) Open STM32CubeProgrammer.
4) Connect the Blue Pill board to your PC using a USB-to-UART converter:
5) TX ↔ RX, RX ↔ TX, GND ↔ GND
6) Set BOOT0 pin to 1 (boot from system memory bootloader).
7) Select UART connection in STM32CubeProgrammer.
8) Load the .bin file and click Download to flash the board.
9) After flashing, set BOOT0 back to 0 and reset the board.

**UPLOADING USING STLINK**
1) Connect the following pins:
  SWDIO → SWDIO
  SWCLK → SWCLK
  3.3V → 3.3V
  GND → GND
2) Ensure BOOT0 is 0
3) In STM32CubeIDE, click:
  Project → Build Project
  Run → Run (or Debug)
4) The code will automatically compile and be flashed to the Blue Pill.
