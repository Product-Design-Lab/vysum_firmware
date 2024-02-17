import argparse
import json
import os
import shutil


CONFIG_FILENAME = "config.json"

DEFAULT_CONFIG = {
    "name": "PDL",
    "email": "name@example.com",
    "lib_path": os.path.join("..", "libraries"),
}

def init_config():
    if os.path.exists(CONFIG_FILENAME):
        print("Config file already exists")
        return

    with open(CONFIG_FILENAME, "w") as f:
        json.dump(DEFAULT_CONFIG, f, indent=4)

    print("Config file created")

def _set_config(key, val):
    if not os.path.exists(CONFIG_FILENAME):
        print("Config file does not exist")
        return

    with open(CONFIG_FILENAME, "r") as f:
        config = json.load(f)

    config[key] = val

    with open(CONFIG_FILENAME, "w") as f:
        json.dump(config, f, indent=4)

    print(f"Config updated: {key} = {val}")

def create_new_library(name):
    # check input is string
    if not isinstance(name, str):
        print("Library name must be a string")
        return

    # read lib_path from file

    config = {}
    try:
        with open(CONFIG_FILENAME, "r") as f:
            try:
                config = json.load(f)
                if ("lib_path" not in config):
                    print("lib_path not set")
                    return
            except json.JSONDecodeError:
                print("Config file is not valid JSON")
                return
    except FileNotFoundError:
        print("Config file does not exist")
        return

    # check for name collision
    lib_path = config["lib_path"]
    new_library_path = os.path.join(lib_path, name)
    if os.path.exists(new_library_path):
        print("Library with same name already exists")
        return

    # create the .cpp and .h files
    src_path = os.path.join(new_library_path, "src")
    os.makedirs(src_path)
    with open(os.path.join(src_path, f"{name}.h"), "w") as f:
        f.write(f'#pragma once\n\n')
    with open(os.path.join(src_path, f"{name}.cpp"), "w") as f:
        f.write(f'#include "{name}.h"\n\n')

    # create example directory
    example_path = os.path.join(new_library_path, "example", "demo")
    os.makedirs(example_path)
    with open(os.path.join(example_path, "demo.ino"), "w") as f:
        f.write(f'#include <Arduino.h>\n')
        f.write(f'#include <{name}.h>\n\n')
        f.write('void setup() \n{\n\n}\n')
        f.write('void loop() \n{\n\n}\n')

    # create library.properties file
    with open(os.path.join(new_library_path, "library.properties"), "w") as f:
        f.write(f'name={name}\n')
        f.write(f'version=0.0.0\n')
        f.write(f'author={config["name"]}\n')
        f.write(f'maintainer={config["name"] } <{config["email"]}>\n')
        f.write(f'sentence=Short description of the library\n')
        f.write(f'paragraph=Longer description of the library\n')
        f.write(f'category=\n')
        f.write(f'url=http://example.com\n')

    # create keywords.txt file
    with open(os.path.join(new_library_path, "keywords.txt"), "w") as f:
        f.write(f'{name} KEYWORD1\n')

    # create README.md file
    with open(os.path.join(new_library_path, "README.md"), "w") as f:
        f.write(f'# {name}\n\n')

    # create lisense file
    with open(os.path.join(new_library_path, "LICENSE"), "w") as f:
        f.write(f'License text\n')
    
    # print new library structure
    print(f"Library created at {new_library_path}")

def config_name(name):
    _set_config("name", name)

def config_email(email):
    _set_config("email", email)

def config_lib_path(path):
    _set_config("lib_path", path)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PDL Development Tools")
    subparsers = parser.add_subparsers(dest="command")

    init_config_parser = subparsers.add_parser(
        "init_config", help="Initialize the config file")

    # python dev_tools.py create_new_library "library_name"
    init_library_parser = subparsers.add_parser(
        "create_new_library", help="Initialize a new library")
    init_library_parser.add_argument("name", help="Name of the library")

    config_parser = subparsers.add_parser(
        "config", help="Set a configuration value")
    config_parser.add_argument("key", choices=["name", "email", "lib_path"])
    config_parser.add_argument("value")

    args = parser.parse_args()

    if args.command == "init_config":
        init_config()
    elif args.command == "create_new_library":
        create_new_library(args.name)
    elif args.command == "config":
        if args.key == "name":
            config_name(args.value)
        elif args.key == "email":
            config_email(args.value)
        elif args.key == "lib_path":
            config_lib_path(args.value)
        else:
            print("Invalid key")
    else:
        parser.print_help()

