# stm32-terrarium-logger
Displays and logs the environmental conditions in a terrarium using a STM32 microcontroller. Currently displays the temperature and humidity measurements taken in two separate terrariums on a pixels matrix LCD screen. 

## Importing Project to STM32CubeIDE
 1. Download and Install STM32CubeIDE from [here](https://www.st.com/en/development-tools/stm32cubeide.html)
 2. Download the source code for this repository
 3. Open STM32CubeIDE, and go to File > Import
 4. Select General > Existing Projects into Workspace, and click 'Next'
 5. Click 'Browse...' next to 'Select root directory:' and select the downloaded source code folder (stm32-terrarium-logger)
 6. Under 'Projects:', select the check box for stm32-terrarium-logger
 7. Click 'Finish'

## Planned Future Features
 - Use an ESP8266 to send logs to a ftp server, or store logs on a removeable, physical storage device (USB drive)
 - Add temperature probes to monitor the soil temperature
 - When the display is off and no measurements are being taken, put the processor in a low-power mode
