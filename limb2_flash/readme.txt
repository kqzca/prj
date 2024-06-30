How to collect data using limb_flash code:

1. Format a SD card (4G or less) with FATFS (FAT16) on PC
2. Create a file named as index.txt on the SD card, write a start number (for example, 0) into that file.
3. Insert SD card into the slot.
4. Power up.
5. After power up, limb_flash code will be in "INIT_NOT_READY" mode. In this mode:
   5.1 External LED flashes at 2Hz (fast).
   5.2 Hardware will be initialized
   5.3 On-board flash device W25Q16 will be erased.
   5.4 It will take ~35 second.
6. Then limb_flash code will be in "READY" mode, waiting for user command. In this mode:
   6.1 External LED is solid ON.
   6.2 Use can move the external switch from "Off" position to "On" position to start data collection.
7. Then limb_flash code will be in "DATA_COLLECTING" mode. In this mode:
   7.1 External LED flashes at 1Hz (normal).
   7.2 limb_flash code will collect data every two milli-seconds.
   7.3 Data will be stored on the on-board flash device W25Q16.
   7.4 Use can move the external switch from "On" position to "Off" position to stop data collection.
   7.5 Or limb_flash code will stop automatically if the on-board flash device W25Q16 is full.
8. Then limb_flash code will be in "WRITING-TO-FILE" mode. In this mode:
   8.1 External LED flashes at 0.5Hz (slow).
   8.2 limb_flash code will read the number in index.txt file, increase it by one, use it for data files.
       For example, if index.txt has '0', the data file will be named as DR000001.CSV
   8.3 New index will be write to index.txt.
   8.4 Data file is in CSV format, which can be imported into MS Excel.
   8.5 Data is human readable raw data, 18 columns per line. Order of the data is:
       - Analog input 1
       - Analog input 2
       - Analog input 3
       - Analog input 4
       - Analog input 5
       - Analog input 6 (Not used)
	   - accel upper x
	   - accel upper y
	   - accel upper z
	   - gyro upper x
	   - gyro upper y
	   - gyro upper z
	   - accel lower x
	   - accel lower y
	   - accel lower z
	   - gyro lower x
	   - gyro lower y
	   - gyro lower z
   8.6 Then limb_flash code will be back in "READY" mode, waiting for user command.

Notes:
1. "Off" position is where input is connected to low/gnd.
2. SD card larger than 4G was not tested.
3. On-board flash device W25Q128 is 128M-bits/16M-bytes in size, may hold data for about 16 minutes.
4. When write to file, 128M data on W25Q128 will generate a file about 36M, a 4G SD card can hold ~100 data files.
6. Attached code can be edited/compiled/downloaded using STM32CubeIDE free version:
   https://www.st.com/en/development-tools/stm32cubeide.html
7. This is the debug interface hardware I use: https://www.st.com/en/development-tools/st-link-v2.html
   A new version of ST-LINK is recommended: https://www.st.com/en/development-tools/stlink-v3set.html