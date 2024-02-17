# PDL - Dev Tools

The dev_tools.py helps you to create template Arduino libraries:

## Windows Usage

1. create default **config.json** file.

    ```batch
    python .\dev_tools.py init_config
    ```

2. Edit the config file.

    ```batch
    python .\dev_tools.py config name "your name"
    python .\dev_tools.py config email "your email"
    python .\dev_tools.py config lib_path "library path"  
    ```

3. Create library.

    ```batch
    python .\dev_tools.py create_new_library "sample_lib"
    ```

4. Navigate to the library folder

    ```batch
    cd ..\libraries\"your library name"
    ```

### Windows Setup Example

```cmd prompt
C:\Users\xl\Documents\vysum_firmware\scripts>python dev_tools.py init_config
Config file already exists

C:\Users\xl\Documents\vysum_firmware\scripts>python dev_tools.py config name "Peter"
Config updated: name = Peter

C:\Users\xl\Documents\vysum_firmware\scripts>python dev_tools.py config email "xutengl@outlook.com"
Config updated: email = xutengl@outlook.com

C:\Users\xl\Documents\vysum_firmware\scripts>python dev_tools.py config lib_path "..\libraries"    
Config updated: libraries_path = ..\libraries

C:\Users\xl\Documents\vysum_firmware\scripts>python dev_tools.py create_new_library "sample_lib"
Library created at ..\libraries\sample_lib
```

## Linux & MacOS Setup

1. Create default **config.json** file.

    ```bash
    python ./dev_tools.py init_config
    ```

2. Edit the config file.

    ```bash
    python dev_tools.py config name "Peter"
    python dev_tools.py config email "your email"
    python dev_tools.py config lib_path "library path"
    ```

3. Create library.

   ```bash
   python dev_tools.py create_new_library "example_lib"
   ```

4. Navigate to the new library directory.

   ```bash
   cd ../libraries
   ```

### Linux & MacOS Example

```bash
xl@XL-NUC MINGW64 ~/Documents/vysum_firmware (dev)
$ cd scripts/

xl@XL-NUC MINGW64 ~/Documents/vysum_firmware/scripts (dev)
$ python dev_tools.py init_config
Config file already exists

xl@XL-NUC MINGW64 ~/Documents/vysum_firmware/scripts (dev)
$ python dev_tools.py config name "Peter"
Config updated: name = Peter

xl@XL-NUC MINGW64 ~/Documents/vysum_firmware/scripts (dev)
$ python dev_tools.py config email "xutengl@outlook.com"
Config updated: email = xutengl@outlook.com

xl@XL-NUC MINGW64 ~/Documents/vysum_firmware/scripts (dev)
$ python dev_tools.py config lib_path "../libraries"
Config updated: libraries_path = ../libraries

xl@XL-NUC MINGW64 ~/Documents/vysum_firmware/scripts (dev)
$ python dev_tools.py create_new_library "sample_lib"
Library with same name already exists

xl@XL-NUC MINGW64 ~/Documents/vysum_firmware/scripts (dev)
$ cd ../libraries/sample_lib/

xl@XL-NUC MINGW64 ~/Documents/vysum_firmware/libraries/sample_lib (dev)
$ ls
example/  keywords.txt  library.properties  LICENSE  README.md  src/
```