# How To setup
Note: you can chick ctrl+shift+v to view this readme file in rendered mode. it's much prettier

- the git submodule init task in task.json requires bash. if using Windows, need to install WSL

- add the following to Files->Preference->Additional boards manager URL:
"https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json"
and go to tools->Boards->Board manager, search and install "Seeed nRF52 Boards". DO NOT install the mbed-enabled alternative. the libraries are not compatible.

- Go to Arduino IDE file->preferences->Settings, set Sketchbook location to the current location, i.e. vysum_firmware/xiao_ble_sense_firmware. This will allow Arduino IDE to download libraries to the libraries/ path, and hence include them in git

- open each project in tests and check that they are all able to compile and run.

# How to add Arduino AppImage to application in Ubuntu
https://forum.arduino.cc/t/arduino-appimage-from-application-on-ubuntu-linux-and-add-to-favorite-menu/1146433

# Recommended VSCode packages:
- hediet.vscode-drawio
- tomoki1207.pdf
- copilot

# Intellisense config
Intellisense is tested on Windows and Ubuntu
1. open a .ino file
2. go to bottom left corner of VSCode window, wait for config intellisense to load
3. choose ubuntu/windows/mac based on your system
4. check if ctrl + click brings you to the definition, and if red squiggle underline disappear
