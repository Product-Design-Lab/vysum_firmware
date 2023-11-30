# How To setup

- add the following to Files->Preference->Additional bards manager URL:
"https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json"
and go to tools->Boards->Board manager, search and install "Seeed nRF52 Baords". DO NOT install the mbed-enabled alternative. the libraries are not compatable. 

- Go to Arduiono IDE file->preferences->Settings, set Sketchbook location to the current location, i.e. vysum_firmware/xiao_ble_sense_firmware. This will allow arduino IDE to download libraries to the libraries/ path, and hence include them in git

- open each projects in tests and check that they are all able to compile and run.

