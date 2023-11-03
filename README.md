# stm32-terrarium-logger
Displays and logs the environmental conditions in a terrarium using a STM32 microcontroller. Currently displays the temperature and humidity measurements taken in two separate terrariums on a pixels matrix LCD screen. 

## Planned Future Features
 - Use an ESP8266 to send logs to a ftp server, or store logs on a removeable, physical storage device (SD card, USB drive)
 - Use DMA with I2C and SPI peripherals, for more efficient use of the CPU
 - Add temperature probes to monitor the soil temperature
 - When the display is off and no measurements are being taken, put the processor in a low-power mode
