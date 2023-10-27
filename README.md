# stm32-terrarium-logger
Displays and logs the environmental conditions in a terrarium using a STM32 microcontroller. Currently displays the temperature and humidity measurements taken in two separate terrariums on a pixels matrix LCD screen. 

## Planned Future Features
 - Turn the backlight on/off based on the amount of light in the room (using a photoresistor)
 - Add timeout to screen to turn off the display after no motion has been detected for a while
 - Use an ESP8266 to send logs to a ftp server, or store logs on a removeable, physical storage device (SD card, USB drive)
 - Use DMA with I2C and SPI peripherals, for more efficient use of the CPU
 - Add "±2%" after the humidity value on screen
 - Add temperature probes to monitor the soil temperature
