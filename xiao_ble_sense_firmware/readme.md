# How To setup

- add the following to Files->Preference->Additional bards manager URL:
"https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json"
and go to tools->Boards->Board manager, search and install "Seeed nRF52 Baords". DO NOT install the mbed-enabled alternative. the libraries are not compatable. 

- Go to Arduiono IDE file->preferences->Settings, set Sketchbook location to the current location, i.e. vysum_firmware/xiao_ble_sense_firmware. This will allow arduino IDE to download libraries to the libraries/ path, and hence include them in git

- open each projects in tests and check that they are all able to compile and run.

# how to add arduino appimage to application in ubuntu
https://forum.arduino.cc/t/arduino-appimage-from-application-on-ubuntu-linux-and-add-to-favorite-menu/1146433

# recommanded vscode packages:
- hediet.vscode-drawio
- tomoki1207.pdf
- 

# enable freertos runtime stat:
- open FreeRTOSConfig.h in the arduino library directory, 
e.g. ~/.arduino15/packages/Seeeduino/hardware/nrf52/1.1.7/cores/nRF5/freertos/config/FreeRTOSConfig.h

- set configGENERATE_RUN_TIME_STATS to 1