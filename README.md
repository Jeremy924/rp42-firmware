# rp42-firmware

The RP-42 firmware can select between
running 3 different programs
1. Executing the current program in Flash (default behavior)
2. Running a hardware test (Hold second button from the top left when starting)
3. Running the command line interface (Hold the top left button when starting)

### 1. Executing the current program
By default, the firmware will just execute whatever software is in the Flash. It does not verify that a program is stored on the flash, so if it is blank or corrupted, it will crash. To do this, the firmware just 
initializes the QSPI in memory mapped mode, loads the initial stack pointer and the starting address from the vector table at 0x9000_0000, and then jumps execution to the starting address. 
### 2. Running a hardware test
Hold the second to the top left button
when first applying power to the calculator. The hardware test only tests the screen and the QSPI. Whenever a button is pressed, it will display that button on the LCD. 
### 3. Running the CLI
To use the CLI through the USB port, hold the top left button down and then apply power to the calculator. 