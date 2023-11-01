# stm32-terrarium-logger
Displays and logs the environmental conditions in a terrarium using a STM32 microcontroller. Currently displays the temperature and humidity measurements taken in two separate terrariums on a pixels matrix LCD screen. 

## Planned Future Features
 - Add a button to 'wake up' the screen (turn on display and backlight), and put screen back to 'sleep' after some time has passed without user input
 - Use an ESP8266 to send logs to a ftp server, or store logs on a removeable, physical storage device (SD card, USB drive)
 - Use DMA with I2C and SPI peripherals, for more efficient use of the CPU
 - Add temperature probes to monitor the soil temperature
